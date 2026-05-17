import heapq
import math
from typing import Sequence

import matplotlib.pyplot as plt
import numpy as np
import shapely.geometry as geometry
from scipy import ndimage


def _xy(pt) -> tuple[float, float]:
    if hasattr(pt, "x") and hasattr(pt, "y"):
        return float(pt.x), float(pt.y)
    if isinstance(pt, (list, tuple)) and len(pt) >= 2:
        return float(pt[0]), float(pt[1])
    raise TypeError(f"Unsupported point type: {type(pt)}")


def _build_safe_geometry(
    boundary_points: Sequence,
    obstacles: Sequence | None,
    clearance: float,
) -> geometry.base.BaseGeometry:
    coords = [_xy(p) for p in boundary_points]
    boundary = geometry.Polygon(coords)
    if boundary.is_empty or not boundary.is_valid:
        raise ValueError("Boundary polygon is empty or invalid.")

    safe = boundary.buffer(-clearance, join_style=2)
    if safe.is_empty:
        raise ValueError(f"clearance={clearance} is too large; safe region becomes empty.")

    if obstacles:
        polys = []
        for ob in obstacles:
            if isinstance(ob, geometry.base.BaseGeometry):
                poly = ob
            else:
                if not isinstance(ob, (list, tuple)) or len(ob) < 3:
                    raise ValueError("Each obstacle must be a polygon (>=3 points) or a bbox with 4 floats.")
                # Bbox: [min_x, min_y, max_x, max_y]
                if len(ob) == 4 and all(isinstance(v, (int, float)) for v in ob):
                    min_x, min_y, max_x, max_y = map(float, ob)
                    if max_x < min_x or max_y < min_y:
                        raise ValueError(f"Invalid bbox with max < min: {ob}")
                    poly = geometry.box(min_x, min_y, max_x, max_y)
                else:
                    coords = [_xy(p) for p in ob]
                    poly = geometry.Polygon(coords)
            if poly.is_empty or not poly.is_valid:
                raise ValueError(f"Obstacle polygon is empty or invalid: {ob}")
            polys.append(poly)
        obstacles_geom = geometry.MultiPolygon(polys).buffer(clearance, join_style=2)
        safe = safe.difference(obstacles_geom)
        if safe.is_empty:
            raise ValueError("Obstacles + clearance remove all free space.")

    return safe


def _rasterize_geometry(
    geom: geometry.base.BaseGeometry,
    resolution: float,
) -> tuple[np.ndarray, float, float]:
    if resolution <= 0:
        raise ValueError("resolution must be > 0")
    minx, miny, maxx, maxy = geom.bounds
    cols = max(1, int(math.ceil((maxx - minx) / resolution)))
    rows = max(1, int(math.ceil((maxy - miny) / resolution)))
    occ = np.ones((rows, cols), dtype=np.uint8)
    for r in range(rows):
        y = miny + (r + 0.5) * resolution
        for c in range(cols):
            x = minx + (c + 0.5) * resolution
            if geom.covers(geometry.Point(x, y)):
                occ[r, c] = 0
    return occ, minx, miny


def _world_to_grid(x: float, y: float, minx: float, miny: float, res: float) -> tuple[int, int]:
    c = int(math.floor((x - minx) / res))
    r = int(math.floor((y - miny) / res))
    return r, c


def _grid_to_world(r: int, c: int, minx: float, miny: float, res: float) -> tuple[float, float]:
    x = minx + (c + 0.5) * res
    y = miny + (r + 0.5) * res
    return round(x, 2), round(y, 2)


def _astar(grid: np.ndarray, start: tuple[int, int], goal: tuple[int, int]):
    h = lambda a, b: abs(a[0] - b[0]) + abs(a[1] - b[1])
    rows, cols = grid.shape
    start = tuple(start)
    goal = tuple(goal)
    open_set = [(h(start, goal), 0, start, None)]
    came_from = {}
    gscore = {start: 0}
    closed = set()
    while open_set:
        _, g, node, parent = heapq.heappop(open_set)
        if node in closed:
            continue
        came_from[node] = parent
        if node == goal:
            path = []
            cur = node
            while cur is not None:
                path.append(cur)
                cur = came_from.get(cur, None)
            path.reverse()
            return path
        closed.add(node)
        x, y = node
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < rows and 0 <= ny < cols and grid[nx, ny] == 0:
                ng = g + 1
                neigh = (nx, ny)
                if neigh in gscore and ng >= gscore[neigh]:
                    continue
                gscore[neigh] = ng
                heapq.heappush(open_set, (ng + h(neigh, goal), ng, neigh, node))
    return None


def _compress_collinear_path(path: list[tuple[int, int]]) -> list[tuple[int, int]]:
    deduped = []
    for point in path:
        if not deduped or point != deduped[-1]:
            deduped.append(point)
    path = deduped

    if len(path) <= 2:
        return path

    compressed = [path[0]]
    prev_dr = path[1][0] - path[0][0]
    prev_dc = path[1][1] - path[0][1]

    for i in range(2, len(path)):
        dr = path[i][0] - path[i - 1][0]
        dc = path[i][1] - path[i - 1][1]
        if (dr, dc) != (prev_dr, prev_dc):
            compressed.append(path[i - 1])
            prev_dr, prev_dc = dr, dc

    compressed.append(path[-1])
    return compressed


def _vertical_boustrophedon_cells(occ: np.ndarray):
    rows, cols = occ.shape
    free_intervals = []
    for c in range(cols):
        col = occ[:, c]
        is_free = (col == 0).astype(int)
        labeled, ncomp = ndimage.label(is_free)
        intervals = []
        for k in range(1, ncomp + 1):
            inds = np.where(labeled == k)[0]
            if inds.size:
                intervals.append((int(inds[0]), int(inds[-1])))
        free_intervals.append(intervals)
    crit = [0]
    prev = free_intervals[0]
    # Only split cells when the number of connected free intervals changes.
    # Endpoint shifts (e.g., diagonal boundaries in a raster) should not
    # create new critical columns, otherwise we get many tiny slabs.
    for c in range(1, cols):
        cur = free_intervals[c]
        if len(cur) != len(prev):
            crit.append(c)
        prev = cur
    crit.append(cols)
    merged = [crit[0]]
    for v in crit[1:]:
        if v - merged[-1] > 1:
            merged.append(v)
    crit = merged
    cells = []
    for i in range(len(crit) - 1):
        x0, x1 = crit[i], crit[i + 1] - 1
        if x1 < x0:
            continue
        slab = occ[:, x0 : x1 + 1]
        free = (slab == 0).astype(np.uint8)
        labeled, n = ndimage.label(free)
        for lab in range(1, n + 1):
            ys, xs = np.where(labeled == lab)
            if ys.size == 0:
                continue
            y0, y1 = int(ys.min()), int(ys.max())
            cells.append(
                {
                    "x0": x0,
                    "x1": x1,
                    "y0": y0,
                    "y1": y1,
                    "centroid": (int((y0 + y1) / 2), int((x0 + x1) / 2)),
                }
            )
    return cells


def _generate_boustrophedon_paths_for_cell(cell, occ: np.ndarray, spacing: int):
    x0, x1, y0, y1 = cell["x0"], cell["x1"], cell["y0"], cell["y1"]
    direction = 1
    path = []

    def append_row(y, sx, ex):
        # add every grid point along the row -> a long straight stroke
        if sx <= ex:
            cols = range(sx, ex + 1)
        else:
            cols = range(sx, ex - 1, -1)
        for c in cols:
            path.append((y, c))

    for y in range(y0, y1 + 1, spacing):   # spacing=1 -> dense
        row = occ[y, x0 : x1 + 1]

        # find free runs in this row
        free_runs = []
        start = None
        for i, v in enumerate(row):
            if v == 0 and start is None:
                start = i
            if v == 1 and start is not None:
                free_runs.append((start, i - 1))
                start = None
        if start is not None:
            free_runs.append((start, len(row) - 1))
        if not free_runs:
            continue

        # pick the longest run (you can also choose “closest to previous” for smoother)
        run = max(free_runs, key=lambda r: r[1] - r[0])
        sx = x0 + run[0]
        ex = x0 + run[1]

        if direction == 1:
            append_row(y, sx, ex)
        else:
            append_row(y, ex, sx)
        direction *= -1

    return path


def _plan_coverage_grid(
    occ: np.ndarray,
    spacing: int,
    start: tuple[int, int] | None,
    goal: tuple[int, int] | None,
):
    cells = _vertical_boustrophedon_cells(occ)
    cell_paths = []
    for c in cells:
        wps = _generate_boustrophedon_paths_for_cell(c, occ, spacing=spacing)
        if wps:
            cell_paths.append({"cell": c, "wps": wps})

    if not cell_paths:
        return cells, cell_paths, []

    if start is None:
        cell_paths.sort(key=lambda cp: (cp["cell"]["centroid"][0], cp["cell"]["centroid"][1]))
        order = cell_paths
    else:
        order = []
        rem = cell_paths.copy()
        cur = start
        while rem:
            best_i = None
            best_d = None
            for i, cp in enumerate(rem):
                cy, cx = cp["cell"]["centroid"]
                d = abs(cur[0] - cy) + abs(cur[1] - cx)
                if best_d is None or d < best_d:
                    best_d = d
                    best_i = i
            next_cp = rem.pop(best_i)
            order.append(next_cp)
            cur = next_cp["cell"]["centroid"]

    def stitch_waypoints(waypoints: list[tuple[int, int]]):
        stitched = []
        for i in range(len(waypoints) - 1):
            seg = _astar(occ, waypoints[i], waypoints[i + 1])
            if not seg:
                raise ValueError(f"Unreachable sweep segment: {waypoints[i]} -> {waypoints[i+1]}")
            if stitched and seg[0] == stitched[-1]:
                stitched.extend(seg[1:])
            else:
                stitched.extend(seg)
        return stitched

    # Build ordered sweep endpoints
    # Build ordered sweep path by concatenation + one A* between cells
    full_path = []
    prev_end = None

    for cp in order:
        wps = cp["wps"]
        if not wps:
            continue

        if prev_end is None:
            # optionally connect start -> first sweep point
            if start is not None and start != wps[0]:
                connector = _astar(occ, start, wps[0])
                if connector is None:
                    raise ValueError("Start cannot reach first sweep point.")
                full_path.extend(connector)
            full_path.extend(wps)
        else:
            connector = _astar(occ, prev_end, wps[0])
            if connector is None:
                raise ValueError(f"Unreachable between cells: {prev_end} -> {wps[0]}")
            # avoid duplicate point
            full_path.extend(connector[1:] if full_path and connector[0] == full_path[-1] else connector)
            full_path.extend(wps)

        prev_end = wps[-1]

    if goal is not None and prev_end is not None and prev_end != goal:
        connector = _astar(occ, prev_end, goal)
        if connector is None:
            raise ValueError("Last sweep point cannot reach goal.")
        full_path.extend(connector[1:] if full_path and connector[0] == full_path[-1] else connector)

    return cells, order, _compress_collinear_path(full_path)


def plan_boustrophedon(
    boundary_points: Sequence, # list of points (x, y) or objects with .x/.y
    obstacles: Sequence | None = None, # list of polygons (>=3 points), shapely geometries, or [min_x, min_y, max_x, max_y] bboxes
    *,
    spacing: float = 1.0,
    clearance: float = 0.0,
    resolution: float = 1.0,
    start=None,
    goal=None,
):
    safe_geom = _build_safe_geometry(boundary_points, obstacles, clearance)
    occ, minx, miny = _rasterize_geometry(safe_geom, resolution)

    spacing_cells = max(1, int(round(spacing / resolution)))

    start_rc = None
    if start is not None:
        sx, sy = _xy(start)
        start_rc = _world_to_grid(sx, sy, minx, miny, resolution)
        if not (0 <= start_rc[0] < occ.shape[0] and 0 <= start_rc[1] < occ.shape[1]):
            raise ValueError("start is outside rasterized bounds.")
        if occ[start_rc] != 0:
            raise ValueError("start is not in free space (after clearance/obstacles).")

    goal_rc = None
    if goal is not None:
        gx, gy = _xy(goal)
        goal_rc = _world_to_grid(gx, gy, minx, miny, resolution)
        if not (0 <= goal_rc[0] < occ.shape[0] and 0 <= goal_rc[1] < occ.shape[1]):
            raise ValueError("goal is outside rasterized bounds.")
        if occ[goal_rc] != 0:
            raise ValueError("goal is not in free space (after clearance/obstacles).")

    cells, cell_paths, full_path_grid = _plan_coverage_grid(
        occ,
        spacing=spacing_cells,
        start=start_rc,
        goal=goal_rc,
    )

    def cell_to_world(cell):
        x0, x1, y0, y1 = cell["x0"], cell["x1"], cell["y0"], cell["y1"]
        wx0 = minx + x0 * resolution
        wx1 = minx + (x1 + 1) * resolution
        wy0 = miny + y0 * resolution
        wy1 = miny + (y1 + 1) * resolution
        cx = (wx0 + wx1) / 2.0
        cy = (wy0 + wy1) / 2.0
        return {"x0": wx0, "x1": wx1, "y0": wy0, "y1": wy1, "centroid": (cx, cy)}

    cells_world = [cell_to_world(c) for c in cells]

    cell_paths_world = []
    for cp in cell_paths:
        wps_world = [
            _grid_to_world(r, c, minx, miny, resolution) for (r, c) in cp["wps"]
        ]
        cell_paths_world.append({"cell": cell_to_world(cp["cell"]), "wps": wps_world})

    full_path_world = [
        _grid_to_world(r, c, minx, miny, resolution) for (r, c) in full_path_grid
    ]

    return cells_world, cell_paths_world, full_path_world


def _plot_demo(boundary_points, obstacles, cells, cell_paths, full_path, start=None, goal=None):
    fig, ax = plt.subplots()
    ax.set_aspect("equal", "box")

    # boundary
    bx, by = zip(*[(_xy(p)[0], _xy(p)[1]) for p in boundary_points])
    ax.fill(bx, by, alpha=0.15, color="lightgray")
    ax.plot(bx + (bx[0],), by + (by[0],), color="black", linewidth=1)

    # obstacles
    if obstacles:
        for ob in obstacles:
            if isinstance(ob, geometry.base.BaseGeometry):
                poly = ob
            else:
                if len(ob) == 4 and all(isinstance(v, (int, float)) for v in ob):
                    min_x, min_y, max_x, max_y = map(float, ob)
                    poly = geometry.box(min_x, min_y, max_x, max_y)
                else:
                    poly = geometry.Polygon([_xy(p) for p in ob])
            if poly.is_empty:
                continue
            if poly.geom_type == "Polygon":
                x, y = poly.exterior.xy
                ax.fill(x, y, alpha=0.3, color="red", edgecolor="darkred")
            else:
                for g in poly.geoms:
                    x, y = g.exterior.xy
                    ax.fill(x, y, alpha=0.3, color="red", edgecolor="darkred")

    # full path
    if full_path:
        fx = [p[0] for p in full_path]
        fy = [p[1] for p in full_path]
        ax.plot(fx, fy, "-r", linewidth=1.2, alpha=0.9)

    if start is not None:
        sx, sy = _xy(start)
        ax.plot(sx, sy, "go")
    if goal is not None:
        gx, gy = _xy(goal)
        ax.plot(gx, gy, "ro")

    ax.set_title("Boustrophedon Coverage (Single Path)")
    plt.show()


if __name__ == "__main__":
    boundary = [
        (0.0, 0.0),
        (12.0, 0.0),
        (12.0, 9.0),
        (7.0, 11.0),
        (0.0, 8.0),
    ]

    obstacles = [
        [2.0, 2.0, 4.5, 4.0],
        [7.0, 1.0, 9.0, 3.5],
        [(5.5, 6.0), (8.0, 6.2), (7.5, 8.5)],
    ]

    start_pt = (1.0, 1.0)
    goal_pt = (11.0, 8.5)
    cells, cell_paths, full_path = plan_boustrophedon(
        boundary,
        obstacles,
        spacing=0.1,
        clearance=0.2,
        resolution=0.1,
        start=start_pt,
        goal=goal_pt,
    )
    
    print(full_path)

    _plot_demo(boundary, obstacles, cells, cell_paths, full_path, start=start_pt, goal=goal_pt)
