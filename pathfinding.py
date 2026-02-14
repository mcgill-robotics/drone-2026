import shapely.geometry as geometry
import matplotlib.pyplot as plt
import math
from typing import Set
import heapq
import itertools

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def __iter__(self):
        yield self.x
        yield self.y

    def __lt__(self, other):
        return (self.x, self.y) < (other.x, other.y)

    def __repr__(self):
        return f"Node({self.x}, {self.y})"
    
    def __eq__(self, value):
        return isinstance(value, Node) and self.x == value.x and self.y == value.y
    
    def __hash__(self):
        return hash((self.x, self.y))
    
class Edge:
    def __init__(self, node1: Node, node2: Node):
        self.node1 = node1
        self.node2 = node2
        self.cost = self.compute_cost()
    
    def compute_cost(self) -> float:
        return math.sqrt((self.node1.x - self.node2.x) ** 2 + (self.node1.y - self.node2.y) ** 2)
    
    def __eq__(self, other):
        if not isinstance(other, Edge):
            return False
        return (
            (self.node1 == other.node1 and self.node2 == other.node2) or
            (self.node1 == other.node2 and self.node2 == other.node1)
        )
    
    def __hash__(self):
        return hash(frozenset((self.node1, self.node2)))

class Graph:
    def __init__(self, nodes: list[Node], edges: list[Edge]):
        self.nodes = nodes
        self.edges = edges
        self.adjancency_list: dict[Node, list[tuple[Node, float]]] = self.build_adjacency_list()
        
    def build_adjacency_list(self) -> dict[Node, list[tuple[Node, float]]]:
        adj_list: dict[Node, list[tuple[Node, float]]] = dict()
        for edge in self.edges:
            l = adj_list.get(edge.node1, [])
            l.append((edge.node2, edge.cost))
            adj_list[edge.node1] = l
            
            l2 = adj_list.get(edge.node2, [])
            l2.append((edge.node1, edge.cost))
            adj_list[edge.node2] = l2
        return adj_list
    
    def get_neighbours(self, node: Node) -> list[tuple[Node, float]]:
        return self.adjancency_list.get(node, [])

class Pathfinding:
    def __init__(self, boundaryPoints: list[Node], waypoints: list[Node], clearance: float = 0.0):
        self.boundary_points = boundaryPoints
        self.waypoints = waypoints
        self.clearance = clearance

        # Original polygon
        coords = [(p.x, p.y) for p in boundaryPoints]
        self.boundary_polygon = geometry.Polygon(coords)

        # Safe polygon
        # join_style=2
        self.safe_polygon = self.boundary_polygon.buffer(-self.clearance, join_style=2)

        if self.safe_polygon.is_empty:
            raise ValueError(f"clearance={clearance} is too large; safe region becomes empty.")

        safe_coords = list(self.safe_polygon.exterior.coords)[:-1]
        self.safe_vertices = [Node(x, y) for (x, y) in safe_coords]

        # Validate waypoints are inside safe region
        for w in self.waypoints:
            if not self.safe_polygon.covers(geometry.Point(w.x, w.y)):
                raise ValueError(f"Waypoint {w} is not inside the safe region (boundary - clearance).")

        # Build visibility graph (nodes + edges)
        self.nodes = self.safe_vertices + self.waypoints
        self.visibility_edges: Set[Edge] = set()
        self._build_visibility_edges()
        self.graph = Graph(self.nodes, list(self.visibility_edges))

    def _segment_is_valid(self, p: Node, q: Node) -> bool:
        line = geometry.LineString([(p.x, p.y), (q.x, q.y)])
        # covers() allows lines on the boundary of safe_polygon, contains() for strictly within
        return self.safe_polygon.covers(line)

    def _build_visibility_edges(self):
        pts = self.nodes
        for i in range(len(pts)):
            for j in range(i + 1, len(pts)):
                p, q = pts[i], pts[j]
                if self._segment_is_valid(p, q):
                    self.visibility_edges.add(Edge(p, q))

    def dijkstra(self, source: Node, dest: Node) -> tuple[float, list[Node]]:
        dist: dict[Node, float] = {source: 0.0}
        prev: dict[Node, Node | None] = {source: None}
        pq: list[tuple[float, Node]] = [(0.0, source)]
        visited: set[Node] = set()

        while pq:
            d, u = heapq.heappop(pq)
            if u in visited:
                continue
            visited.add(u)

            if u == dest:
                break

            for v, w in self.graph.get_neighbours(u):
                nd = d + w
                if nd < dist.get(v, float("inf")):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))

        if dest not in dist:
            return float("inf"), []

        # Reconstruct path
        path: list[Node] = []
        cur: Node | None = dest
        while cur is not None:
            path.append(cur)
            cur = prev.get(cur)
        path.reverse()
        return dist[dest], path

    def tsp_loop_order(self) -> list[Node]:
        """
        Returns an ordered list of waypoints (start fixed at index 0),
        representing a cycle: start -> ... -> start.
        Uses Held–Karp DP (exact) for moderate N.
        """
        pts = self.waypoints
        n = len(pts)
        if n < 2:
            return pts + pts[:1]

        # Pairwise shortest-path distances between waypoints via visibility graph
        dist = [[0.0] * n for _ in range(n)]
        for i in range(n):
            for j in range(n):
                if i == j:
                    continue
                d, _ = self.dijkstra(pts[i], pts[j])
                dist[i][j] = d

        # Held–Karp with fixed start = 0
        # DP[(mask, j)] = best cost from start to j visiting subset mask (mask includes j, excludes start)
        DP: dict[tuple[int, int], float] = {}
        parent: dict[tuple[int, int], int] = {}

        # initialize: go from start(0) to j
        for j in range(1, n):
            DP[(1 << j, j)] = dist[0][j]
            parent[(1 << j, j)] = 0

        # fill
        for r in range(2, n):
            for subset in itertools.combinations(range(1, n), r):
                mask = 0
                for b in subset:
                    mask |= 1 << b
                for j in subset:
                    pmask = mask ^ (1 << j)
                    best = float("inf")
                    best_k = -1
                    for k in subset:
                        if k == j:
                            continue
                        cand = DP.get((pmask, k), float("inf")) + dist[k][j]
                        if cand < best:
                            best = cand
                            best_k = k
                    DP[(mask, j)] = best
                    parent[(mask, j)] = best_k

        # close the tour back to start
        full_mask = 0
        for j in range(1, n):
            full_mask |= 1 << j

        best_cost = float("inf")
        last = -1
        for j in range(1, n):
            cand = DP.get((full_mask, j), float("inf")) + dist[j][0]
            if cand < best_cost:
                best_cost = cand
                last = j

        if last == -1 or math.isinf(best_cost):
            raise ValueError("No feasible TSP tour: at least one waypoint pair is unreachable in the safe region.")

        # reconstruct order (indices)
        order = [0]
        mask = full_mask
        j = last
        rev = [j]
        while True:
            pj = parent[(mask, j)]
            if pj == 0:
                break
            rev.append(pj)
            mask ^= 1 << j
            j = pj
        order += list(reversed(rev))
        order.append(0)  # return to start
        return [pts[i] for i in order]

    def tsp_loop_path(self) -> list[Node]:
        """
        Returns the actual geometric loop as a list of Nodes, stitching shortest paths.
        """
        ordered = self.tsp_loop_order()
        stitched: list[Node] = []
        for i in range(len(ordered) - 1):
            _, seg = self.dijkstra(ordered[i], ordered[i + 1])
            if not seg:
                raise ValueError(f"Unreachable segment: {ordered[i]} -> {ordered[i+1]}")
            if stitched and seg[0] == stitched[-1]:
                stitched.extend(seg[1:])
            else:
                stitched.extend(seg)
        return stitched

    def plot(self, show_safe=True, show_waypoints=True):
        _, ax = plt.subplots()
        ax.set_aspect("equal", "box")

        # original boundary
        x, y = self.boundary_polygon.exterior.xy
        ax.fill(x, y, alpha=0.2, color="red")

        # safe region
        if show_safe:
            sx, sy = self.safe_polygon.exterior.xy
            ax.plot(sx, sy, color="black", alpha=0.8)

        # visibility edges
        for e in self.visibility_edges:
            p, q = e.node1, e.node2
            ax.plot([p.x, q.x], [p.y, q.y], color="blue", alpha=0.25)

        # waypoints
        if show_waypoints:
            for w in self.waypoints:
                ax.plot(w.x, w.y, "go")

        plt.show()

    def plot_tsp_loop(self):
        loop = self.tsp_loop_path()
        _, ax = plt.subplots()
        ax.set_aspect("equal", "box")

        x, y = self.boundary_polygon.exterior.xy
        ax.fill(x, y, alpha=0.2, color="red")

        sx, sy = self.safe_polygon.exterior.xy
        ax.plot(sx, sy, color="black", alpha=0.8)

        # draw the loop polyline
        for i in range(len(loop) - 1):
            p, q = loop[i], loop[i + 1]
            ax.plot([p.x, q.x], [p.y, q.y], alpha=0.9)

        for w in self.waypoints:
            ax.plot(w.x, w.y, "go")

        plt.show()


# ---- Testing ----
boundary = [
    Node(0, 0),
    Node(10, 0),
    Node(10, 10),
    Node(0, 10),
    Node(5,5)
]

waypoints = [
    Node(2, 1),   # start (fixed)
    Node(2, 9),
    Node(7, 3),
    Node(9, 9)
]

pf = Pathfinding(boundary, waypoints, clearance=0.5)
pf.plot()
print("TSP order:", pf.tsp_loop_order())
pf.plot_tsp_loop()
