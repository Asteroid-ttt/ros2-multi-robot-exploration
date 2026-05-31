"""
RACER-inspired 2D Exploration Module for Multi-Robot Ground Vehicles
===========================================================================
Adapts key ideas from the RACER (Rapid Collaborative Exploration) framework
to 2D ground robots using Nav2 for navigation.

Key improvements over simple frontier-based exploration:
  1. Viewpoint Sampling     — Multiple candidate observation positions per frontier
  2. Visibility Raycasting  — 2D raycasting to estimate information gain per viewpoint
  3. Path-aware Cost Matrix — A* grid distances instead of Euclidean heuristic
  4. Tour Optimization      — 2-opt TSP tour refinement for each robot
  5. Grid Partitioning      — 2-level hierarchical grid for work division
  6. Pairwise Coordination  — Conflict avoidance between robots

Reference: RACER — Rapid Collaborative Exploration with a Decentralized
           Multi-UAV System (IEEE TRO 2023 Best Paper)
"""

import numpy as np
import heapq
import math
from collections import deque
from itertools import permutations


# ============================================================================
# Configuration Parameters
# ============================================================================

class ExplorerConfig:
    """Centralized configuration for the RACER 2D explorer."""

    # --- Costmap ---
    expansion_size: int = 7          # Obstacle inflation radius (cells)
    lethal_cost: int = 100           # Occupied cell value

    # --- Frontier Detection ---
    min_frontier_size: int = 3       # Minimum cells in a frontier cluster
    max_frontier_clusters: int = 10  # Max frontier clusters to consider
    pca_split_threshold: float = 0.7 # Variance ratio threshold for PCA split

    # --- Viewpoint Sampling ---
    viewpoint_samples: int = 8       # Angular samples around frontier
    viewpoint_radius_min: float = 1.5  # Min distance from frontier (meters)
    viewpoint_radius_max: float = 3.0  # Max distance from frontier (meters)
    viewpoint_free_check: bool = True  # Require viewpoint on free cell
    max_viewpoints_per_frontier: int = 3  # Top-K viewpoints by visibility

    # --- Visibility (2D Raycasting) ---
    ray_range: float = 4.0           # Max sensor range for raycasting (meters)
    ray_angular_resolution: int = 36 # Number of rays (360° / resolution)

    # --- Path Cost ---
    max_tsp_frontiers: int = 8       # Max frontiers in TSP tour
    use_path_distance: bool = True   # Use A* distance vs Euclidean

    # --- Tour Planning ---
    tour_horizon: int = 3            # How many waypoints to plan ahead

    # --- Grid Partitioning ---
    grid_level1_size: float = 10.0   # Coarse grid cell size (meters)
    grid_level2_size: float = 4.0    # Fine grid cell size (meters)

    # --- Coordination ---
    robot_conflict_radius: float = 2.0  # Min distance between robot targets
    visited_radius: float = 0.5      # Radius to mark a target as visited
    max_target_distance: float = 150.0 # Max acceptable target distance


# ============================================================================
# 1. Costmap Construction
# ============================================================================

def build_costmap(occupancy_data, width, height, expansion_size=7):
    """
    Inflate obstacles to create a costmap for safe planning.

    Args:
        occupancy_data: Raw occupancy grid (1D numpy array)
        width, height: Grid dimensions
        expansion_size: Obstacle inflation radius in cells

    Returns:
        2D numpy array: inflated costmap (0=free, 100=obstacle, -1=unknown)
    """
    data = np.array(occupancy_data, dtype=np.float64).reshape(height, width)
    obstacle_cells = np.where(data == 100)

    for i in range(-expansion_size, expansion_size + 1):
        for j in range(-expansion_size, expansion_size + 1):
            if i == 0 and j == 0:
                continue
            x = np.clip(obstacle_cells[0] + i, 0, height - 1)
            y = np.clip(obstacle_cells[1] + j, 0, width - 1)
            data[x, y] = 100

    return data


# ============================================================================
# 2. Frontier Detection (RACER-style: Region Growing + PCA Split)
# ============================================================================

def detect_frontiers(grid, min_size=3):
    """
    Detect frontier cells (free cells adjacent to unknown) and cluster them
    using 8-connected region growing.

    A frontier cell is a FREE cell (0) that has at least one UNKNOWN neighbor (-1).

    Args:
        grid: 2D occupancy grid (0=free, 100=obstacle, -1=unknown)
        min_size: Minimum number of cells for a valid frontier cluster

    Returns:
        list of dicts: [{cells: [(r,c),...], centroid: (r,c), bbox: (min_r,min_c,max_r,max_c)}, ...]
    """
    height, width = grid.shape
    visited = np.zeros_like(grid, dtype=bool)
    frontiers = []

    for r in range(height):
        for c in range(width):
            if grid[r, c] != 0 or visited[r, c]:
                continue
            # Check if adjacent to unknown
            has_unknown = False
            for dr, dc in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < height and 0 <= nc < width and grid[nr, nc] == -1:
                    has_unknown = True
                    break
            if not has_unknown:
                continue

            # Region growing (BFS, 8-connected)
            cluster = []
            q = deque([(r, c)])
            visited[r, c] = True
            min_r, min_c = r, c
            max_r, max_c = r, c

            while q:
                cr, cc = q.popleft()
                cluster.append((cr, cc))
                min_r, min_c = min(min_r, cr), min(min_c, cc)
                max_r, max_c = max(max_r, cr), max(max_c, cc)

                for dr, dc in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                    nr, nc = cr + dr, cc + dc
                    if 0 <= nr < height and 0 <= nc < width:
                        if grid[nr, nc] == 0 and not visited[nr, nc]:
                            # Only grow into frontier cells (free + adjacent to unknown)
                            is_frontier = False
                            for dr2, dc2 in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                                nnr, nnc = nr + dr2, nc + dc2
                                if 0 <= nnr < height and 0 <= nnc < width and grid[nnr, nnc] == -1:
                                    is_frontier = True
                                    break
                            if is_frontier:
                                visited[nr, nc] = True
                                q.append((nr, nc))

            if len(cluster) >= min_size:
                centroid_r = sum(c[0] for c in cluster) / len(cluster)
                centroid_c = sum(c[1] for c in cluster) / len(cluster)
                frontiers.append({
                    'cells': cluster,
                    'centroid': (centroid_r, centroid_c),
                    'bbox': (min_r, min_c, max_r, max_c),
                    'size': len(cluster),
                })

    # Split large frontiers using PCA
    frontiers = _split_large_frontiers(frontiers, grid)

    # Sort by size descending
    frontiers.sort(key=lambda f: f['size'], reverse=True)
    return frontiers


def _split_large_frontiers(frontiers, grid, variance_threshold=0.7):
    """
    Split large frontier clusters that span a wide area using PCA.
    If the variance along the principal component is > threshold of total variance,
    split the cluster into two along the principal axis.
    """
    result = []
    for f in frontiers:
        cells = f['cells']
        if len(cells) < 12:  # Too small to split meaningfully
            result.append(f)
            continue

        pts = np.array(cells, dtype=np.float64)
        mean = pts.mean(axis=0)
        centered = pts - mean
        cov = centered.T @ centered / (len(pts) - 1)

        try:
            eigenvalues, eigenvectors = np.linalg.eig(cov)
            idx = np.argsort(eigenvalues)[::-1]
            eigenvalues = eigenvalues[idx]
            eigenvectors = eigenvectors[:, idx]

            # If variance along PC1 is dominant, split
            if eigenvalues[0] / eigenvalues.sum() > variance_threshold:
                pc1 = eigenvectors[:, 0]
                projections = centered @ pc1

                # Split at median projection
                median_proj = np.median(projections)
                group_a = [cells[i] for i in range(len(cells)) if projections[i] <= median_proj]
                group_b = [cells[i] for i in range(len(cells)) if projections[i] > median_proj]

                if len(group_a) >= 3 and len(group_b) >= 3:
                    for g in [group_a, group_b]:
                        ca = sum(c[0] for c in g) / len(g)
                        cb = sum(c[1] for c in g) / len(g)
                        min_r = min(c[0] for c in g)
                        min_c = min(c[1] for c in g)
                        max_r = max(c[0] for c in g)
                        max_c = max(c[1] for c in g)
                        result.append({
                            'cells': g,
                            'centroid': (ca, cb),
                            'bbox': (min_r, min_c, max_r, max_c),
                            'size': len(g),
                        })
                    continue
        except np.linalg.LinAlgError:
            pass

        result.append(f)
    return result


# ============================================================================
# 3. Viewpoint Sampling (RACER-style)
# ============================================================================

def sample_viewpoints(frontier, costmap, resolution, origin,
                      num_samples=8, radius_min=1.5, radius_max=3.0,
                      top_k=3, ray_range=4.0, ray_resolution=36):
    """
    Generate multiple candidate viewpoints around a frontier cluster and
    evaluate each by 2D raycasting visibility.

    Args:
        frontier: Frontier dict from detect_frontiers()
        costmap: 2D occupancy grid (0=free, 100=obstacle, -1=unknown)
        resolution: Grid resolution (m/cell)
        origin: (origin_x, origin_y) of grid in world frame
        num_samples, radius_min, radius_max: Viewpoint sampling params
        top_k: Number of best viewpoints to return
        ray_range, ray_resolution: Raycasting params

    Returns:
        list of dicts: [{world_pos: (x,y), grid_pos: (r,c), visibility: int, yaw: float}, ...]
    """
    height, width = costmap.shape
    cr, cc = frontier['centroid']
    frontier_cells = set(frontier['cells'])

    # Sample positions on concentric circles around frontier centroid
    viewpoints = []
    radii = np.linspace(radius_min / resolution, radius_max / resolution, 3)

    for radius in radii:
        angles = np.linspace(0, 2 * math.pi, num_samples, endpoint=False)
        for angle in angles:
            vr = int(cr + radius * math.cos(angle))
            vc = int(cc + radius * math.sin(angle))

            # Validate viewpoint is in free space and not too close to unknown
            if not (0 <= vr < height and 0 <= vc < width):
                continue
            if costmap[vr, vc] != 0:
                continue

            # Check not too close to unknown cells (safety margin)
            too_close = False
            for dr in range(-2, 3):
                for dc in range(-2, 3):
                    nr, nc = vr + dr, vc + dc
                    if 0 <= nr < height and 0 <= nc < width and costmap[nr, nc] == -1:
                        too_close = True
                        break
                if too_close:
                    break
            if too_close:
                continue

            # 2D raycasting visibility evaluation
            vis_count, best_yaw = _evaluate_visibility_2d(
                (vr, vc), frontier_cells, costmap, ray_range, resolution, ray_resolution
            )

            if vis_count > 0:
                world_x = vc * resolution + origin[0]
                world_y = vr * resolution + origin[1]
                viewpoints.append({
                    'world_pos': (world_x, world_y),
                    'grid_pos': (vr, vc),
                    'visibility': vis_count,
                    'yaw': best_yaw,
                })

    # Sort by visibility and return top-k
    viewpoints.sort(key=lambda v: v['visibility'], reverse=True)
    return viewpoints[:top_k]


def _evaluate_visibility_2d(viewpoint, frontier_cells, costmap, ray_range, resolution, num_rays=36):
    """
    Evaluate how many frontier cells are visible from a viewpoint using 2D raycasting.

    Args:
        viewpoint: (row, col) of viewpoint in grid coordinates
        frontier_cells: set of (row, col) frontier cells
        costmap: 2D occupancy grid
        ray_range: max ray length in meters
        resolution: m/cell
        num_rays: number of rays to cast

    Returns:
        (visible_count, best_yaw_angle): number of visible frontier cells, best yaw
    """
    vr, vc = viewpoint
    max_steps = int(ray_range / resolution)
    height, width = costmap.shape

    # For 2D ground robots, we evaluate rays in all directions
    # and aggregate visibility per direction bin
    vis_by_direction = {}
    total_visible = set()

    for i in range(num_rays):
        angle = 2 * math.pi * i / num_rays
        dr = math.cos(angle)
        dc = math.sin(angle)

        seen_cells = set()
        for step in range(1, max_steps + 1):
            r = int(vr + step * dr)
            c = int(vc + step * dc)

            if not (0 <= r < height and 0 <= c < width):
                break

            cell_val = costmap[r, c]
            if cell_val == 100:  # Hit obstacle
                # Mark obstacle edge cell if it's a frontier
                if (r, c) in frontier_cells:
                    seen_cells.add((r, c))
                break
            elif cell_val == -1:  # Unknown
                seen_cells.add((r, c))
                break  # Unknown blocks the ray
            elif cell_val == 0:  # Free
                if (r, c) in frontier_cells:
                    seen_cells.add((r, c))
                # Continue through free space

        # Bin into 8 direction sectors
        sector = int(((angle + math.pi) / (2 * math.pi) * 8)) % 8
        if sector not in vis_by_direction:
            vis_by_direction[sector] = set()
        vis_by_direction[sector].update(seen_cells)
        total_visible.update(seen_cells)

    # Find best yaw (direction with max visibility)
    best_yaw = 0.0
    max_vis = 0
    for sector, cells in vis_by_direction.items():
        if len(cells) > max_vis:
            max_vis = len(cells)
            best_yaw = sector * math.pi / 4  # Convert sector to angle

    return len(total_visible), best_yaw


# ============================================================================
# 4. Path Cost Computation (A* on Grid)
# ============================================================================

def astar_path_distance(grid, start, goal):
    """
    Compute shortest path distance on the occupancy grid using A*.

    Args:
        grid: 2D occupancy grid (0=free, 100=obstacle, -1=unknown treated as free for planning)
        start: (row, col) start position
        goal: (row, col) goal position

    Returns:
        float: path distance in grid cells, or infinity if no path exists
    """
    height, width = grid.shape
    sr, sc = start
    gr, gc = goal

    # 8-connected movement
    moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
             (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]

    def heuristic(r, c):
        return math.sqrt((r - gr) ** 2 + (c - gc) ** 2)

    open_set = [(heuristic(sr, sc), 0, sr, sc)]
    g_scores = {(sr, sc): 0.0}
    visited = set()

    while open_set:
        f, g, r, c = heapq.heappop(open_set)

        if (r, c) in visited:
            continue
        visited.add((r, c))

        if r == gr and c == gc:
            return g

        for dr, dc, cost in moves:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < height and 0 <= nc < width):
                continue
            if grid[nr, nc] == 100:  # Obstacle
                continue

            ng = g + cost
            if ng < g_scores.get((nr, nc), float('inf')):
                g_scores[(nr, nc)] = ng
                heapq.heappush(open_set, (ng + heuristic(nr, nc), ng, nr, nc))

    return float('inf')


# ============================================================================
# 5. Tour Planning (Greedy TSP + 2-opt Refinement)
# ============================================================================

def plan_tour(robot_grid_pos, frontiers, costmap, resolution, origin,
              max_frontiers=8, tour_horizon=3):
    """
    Plan an optimized tour through the most promising frontiers for a single robot.

    Steps:
      1. Select top-K frontiers by weighted score (size / distance)
      2. Sample viewpoints for each frontier
      3. Build cost matrix (path-aware)
      4. Solve TSP tour (greedy + 2-opt)
      5. Return top N waypoints

    Args:
        robot_grid_pos: (row, col) of robot in grid coordinates
        frontiers: List of frontier dicts
        costmap: 2D occupancy grid
        resolution: m/cell
        origin: (ox, oy) world origin
        max_frontiers: Max frontiers to consider
        tour_horizon: Number of waypoints to return

    Returns:
        list of (world_x, world_y): Ordered tour waypoints
    """
    if not frontiers:
        return []

    # Step 1: Score and select top frontiers
    scored = []
    for f in frontiers[:max_frontiers]:
        d = math.sqrt((robot_grid_pos[0] - f['centroid'][0]) ** 2 +
                      (robot_grid_pos[1] - f['centroid'][1]) ** 2) + 1e-6
        score = f['size'] / d
        scored.append((score, f))

    scored.sort(key=lambda x: x[0], reverse=True)
    selected = [s[1] for s in scored[:max_frontiers]]

    if not selected:
        return []

    # Step 2: Sample best viewpoint for each selected frontier
    frontier_viewpoints = []
    for f in selected:
        vps = sample_viewpoints(f, costmap, resolution, origin)
        if vps:
            frontier_viewpoints.append(vps[0])  # Best viewpoint per frontier
        else:
            # Fallback to centroid
            wx = f['centroid'][1] * resolution + origin[0]
            wy = f['centroid'][0] * resolution + origin[1]
            frontier_viewpoints.append({
                'world_pos': (wx, wy),
                'grid_pos': (int(f['centroid'][0]), int(f['centroid'][1])),
                'visibility': f['size'],
                'yaw': 0.0,
            })

    if not frontier_viewpoints:
        return []

    # Step 3: Build cost matrix (path distances)
    n = len(frontier_viewpoints)
    cost_matrix = np.zeros((n + 1, n + 1))  # +1 for robot start

    # Robot start to each viewpoint
    for j, vp in enumerate(frontier_viewpoints):
        d = astar_path_distance(costmap, robot_grid_pos, vp['grid_pos'])
        cost_matrix[0, j + 1] = d if d != float('inf') else 1e6

    # Between viewpoints
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            d = astar_path_distance(costmap,
                                    frontier_viewpoints[i]['grid_pos'],
                                    frontier_viewpoints[j]['grid_pos'])
            cost_matrix[i + 1, j + 1] = d if d != float('inf') else 1e6

    # Step 4: Greedy TSP tour from robot position
    tour = _greedy_tsp(cost_matrix, n)

    # Step 5: 2-opt refinement
    tour = _two_opt(tour, cost_matrix, max_iterations=50)

    # Step 6: Extract first N waypoints
    waypoints = []
    for idx in tour[:tour_horizon]:
        if idx > 0:  # idx 0 is robot position
            vp = frontier_viewpoints[idx - 1]
            waypoints.append(vp['world_pos'])

    return waypoints


def _greedy_tsp(cost_matrix, n):
    """Greedy nearest-neighbor TSP tour starting from position 0."""
    unvisited = set(range(1, n + 1))
    tour = [0]
    current = 0

    while unvisited:
        best_next = min(unvisited, key=lambda j: cost_matrix[current, j])
        tour.append(best_next)
        unvisited.remove(best_next)
        current = best_next

    return tour


def _two_opt(tour, cost_matrix, max_iterations=50):
    """2-opt local search to improve a TSP tour."""
    improved = True
    iteration = 0

    while improved and iteration < max_iterations:
        improved = False
        iteration += 1

        for i in range(1, len(tour) - 2):
            for j in range(i + 1, len(tour) - 1):
                # Check if swapping edges (i-1,i) and (j,j+1) improves tour
                old_cost = (cost_matrix[tour[i - 1], tour[i]] +
                            cost_matrix[tour[j], tour[j + 1]])
                new_cost = (cost_matrix[tour[i - 1], tour[j]] +
                            cost_matrix[tour[i], tour[j + 1]])

                if new_cost < old_cost - 1e-6:
                    tour[i:j + 1] = reversed(tour[i:j + 1])
                    improved = True

    return tour


# ============================================================================
# 6. Hierarchical Grid Partitioning
# ============================================================================

def partition_frontiers_by_grid(frontiers, robot_positions, costmap, resolution, origin,
                                 grid_size=10.0):
    """
    Partition frontiers into spatial grid cells and assign to robots.

    This implements RACER's hierarchical grid partitioning adapted for 2D:
      - Divide the explored space into grid cells
      - Assign each frontier to a grid cell
      - For each robot, recommend nearby grid cells to explore

    Args:
        frontiers: List of frontier dicts
        robot_positions: dict {robot_name: (world_x, world_y)}
        costmap: 2D occupancy grid
        resolution: m/cell
        origin: (ox, oy)
        grid_size: Cell size in meters

    Returns:
        dict: {robot_name: [frontier, ...]} assigned frontiers per robot
    """
    if not frontiers or not robot_positions:
        return {name: frontiers for name in robot_positions}

    grid_cells = {}  # (gx, gy) -> [frontiers]
    for f in frontiers:
        wx = f['centroid'][1] * resolution + origin[0]
        wy = f['centroid'][0] * resolution + origin[1]
        gx = int(wx / grid_size)
        gy = int(wy / grid_size)
        key = (gx, gy)
        if key not in grid_cells:
            grid_cells[key] = []
        grid_cells[key].append(f)

    # Assign grid cells to nearest robot
    robot_names = list(robot_positions.keys())
    assignment = {name: [] for name in robot_names}

    for (gx, gy), cell_frontiers in grid_cells.items():
        cell_center = ((gx + 0.5) * grid_size, (gy + 0.5) * grid_size)

        # Find nearest robot
        best_robot = min(robot_names,
                         key=lambda n: math.sqrt(
                             (robot_positions[n][0] - cell_center[0]) ** 2 +
                             (robot_positions[n][1] - cell_center[1]) ** 2))
        assignment[best_robot].extend(cell_frontiers)

    return assignment


# ============================================================================
# 7. Pairwise Coordination (Conflict Avoidance)
# ============================================================================

def resolve_target_conflicts(robot_targets, conflict_radius=2.0):
    """
    Detect and resolve conflicts when two robots are assigned the same
    or very close targets.

    Strategy: If two targets are within conflict_radius, keep the assignment
    for the robot that is closer, and reassign the other.

    Args:
        robot_targets: dict {robot_name: (world_x, world_y) or None}
        conflict_radius: Minimum distance between robot targets (meters)

    Returns:
        dict: Updated robot_targets with conflicts resolved
    """
    resolved = dict(robot_targets)
    robot_names = [n for n, t in resolved.items() if t is not None]

    for i in range(len(robot_names)):
        for j in range(i + 1, len(robot_names)):
            ni, nj = robot_names[i], robot_names[j]
            ti, tj = resolved[ni], resolved[nj]
            if ti is None or tj is None:
                continue

            dist = math.sqrt((ti[0] - tj[0]) ** 2 + (ti[1] - tj[1]) ** 2)
            if dist < conflict_radius:
                # The farther robot loses its target
                # (In a more advanced implementation, we'd reassign to next-best)
                # For now, keep the one with smaller index
                resolved[nj] = None

    return resolved


# ============================================================================
# 8. Main Exploration Function (RACER-style pipeline)
# ============================================================================

def racer_explore(map_data, map_width, map_height, resolution, origin_x, origin_y,
                  robot_positions, robot_name, last_target=None,
                  config: ExplorerConfig = None):
    """
    RACER-style exploration pipeline for a single robot.

    Full pipeline:
      1. Build inflated costmap
      2. Detect frontiers (region growing + PCA split)
      3. Partition frontiers by spatial grid → per-robot assignment
      4. Plan optimized tour (viewpoint sampling + TSP)
      5. Return next best waypoint

    Args:
        map_data: 2D numpy occupancy grid
        map_width, map_height: Grid dimensions
        resolution: m/cell
        origin_x, origin_y: World origin of grid
        robot_positions: dict {name: (world_x, world_y)} of all robots
        robot_name: Name of this robot
        last_target: Previous target (to avoid repeats)
        config: ExplorerConfig instance

    Returns:
        (target_x, target_y) or None if no valid target
    """
    if config is None:
        config = ExplorerConfig()

    height, width = map_data.shape

    # Step 1: Build costmap
    costmap = build_costmap(map_data.flatten(), width, height, config.expansion_size)

    # Mark current robot position as free on costmap
    rx_world = robot_positions.get(robot_name, (0.0, 0.0))
    rx_grid = int((rx_world[1] - origin_y) / resolution)
    ry_grid = int((rx_world[0] - origin_x) / resolution)
    if 0 <= rx_grid < width and 0 <= ry_grid < height:
        costmap[ry_grid, rx_grid] = 0

    # Normalize: unknown=-1, free=0, obstacle=100
    costmap[(costmap > 5) & (costmap < 100)] = 0

    # Step 2: Detect frontiers
    frontiers = detect_frontiers(costmap, min_size=config.min_frontier_size)
    frontiers = frontiers[:config.max_frontier_clusters]

    if not frontiers:
        return None

    # Step 3: Partition frontiers per robot
    assignment = partition_frontiers_by_grid(
        frontiers, robot_positions, costmap, resolution, (origin_x, origin_y),
        grid_size=config.grid_level1_size
    )
    my_frontiers = assignment.get(robot_name, frontiers)

    # Step 4: Plan tour
    robot_grid_pos = (ry_grid, rx_grid)  # (row, col)
    waypoints = plan_tour(
        robot_grid_pos, my_frontiers, costmap, resolution, (origin_x, origin_y),
        max_frontiers=config.max_tsp_frontiers,
        tour_horizon=config.tour_horizon
    )

    if not waypoints:
        return None

    target = waypoints[0]

    # Avoid sending the same target repeatedly
    if last_target is not None:
        d = math.sqrt((target[0] - last_target[0]) ** 2 + (target[1] - last_target[1]) ** 2)
        if d < config.visited_radius:
            if len(waypoints) > 1:
                target = waypoints[1]
            else:
                return None

    # Distance sanity check
    dist_to_target = math.sqrt((target[0] - rx_world[0]) ** 2 + (target[1] - rx_world[1]) ** 2)
    if dist_to_target > config.max_target_distance:
        return None

    return target


# ============================================================================
# 9. Utility: Visualization Helpers
# ============================================================================

def frontier_grid_to_display(grid, frontiers):
    """
    Create a display grid with frontiers highlighted.
    0=free, 1=obstacle, 2=frontier, 3=unknown
    """
    display = np.zeros_like(grid, dtype=np.int32)
    display[grid == 100] = 1
    display[grid == -1] = 3

    for f in frontiers:
        for r, c in f['cells']:
            display[r, c] = 2

    return display
