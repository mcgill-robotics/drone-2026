import shapely.geometry as geometry
import matplotlib.pyplot as plt
import math
from typing import Set
import queue

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
        return self.x == value.x and self.y == value.y
    
class Edge:
    def __init__(self, node1 : Node, node2 : Node):
        self.node1 = node1
        self.node2 = node2
        self.cost = self.compute_cost()
    
    def compute_cost(self) -> float:
        return math.sqrt((self.node1.x - self.node2.x) ** 2 + (self.node1.y - self.node2.y) ** 2)
    
    def __eq__(self, other : object):
        return self.node1 == other.node1 and self.node2 == other.node2
    
class Graph:
    def __init__(self, nodes : list[Node], edges : list[Edge]):
        self.nodes = nodes
        self.edges = edges
        self.adjancency_list : dict[Node, list[tuple[Node, float]]] = self.build_adjacency_list()
        
    def build_adjacency_list(self) -> dict[Node, list[tuple[Node, float]]]:
        adj_list : dict[Node, list[tuple[Node, float]]] = dict()
        for edge in self.edges:
            l = adj_list.get(edge.node1, [])
            l.append((edge.node2, edge.cost))
            adj_list[edge.node1] = l
            
            l2 = adj_list.get(edge.node2, [])
            l2.append((edge.node1, edge.cost))
            adj_list[edge.node2] = l2
        return adj_list
    
    def get_neighbours(self, node : Node) -> list[tuple[Node, float]]:
        return self.adjancency_list[node]
            

class Pathfinding:
    def __init__(self, boundaryPoints: list[Node], waypoints: list[Node]):
        self.boundary_points = boundaryPoints
        self.waypoints = waypoints
        
        self.boundary_segments = []
        for i in range(len(boundaryPoints)):
            # TODO: extend segments by safe distance
            self.boundary_segments.append((boundaryPoints[i-1], boundaryPoints[i]))
        coords = [(p.x, p.y) for p in boundaryPoints]
        self.boundary_polygon = geometry.Polygon(coords)
        
        points = boundaryPoints + waypoints
        self.visibility_edges : Set[Edge] = set()
        
        for i in range(len(points)):
            for j in range(i+1, len(points)):
                p = points[i]
                q = points[j]
                line = geometry.LineString([(p.x, p.y), (q.x, q.y)])
                if (line.touches(self.boundary_polygon) == False):
                    self.visibility_edges.add(Edge(p, q))
        return
    
    def pathfind(self):
        # TODO
        path = [self.waypoints[0]]
        for i in range(len(self.waypoints) - 1):
            edge = Edge(waypoints[i], waypoints[i+1])
            if edge in self.visibility_edges:
                path.append(waypoints[i+1])
                continue
            else:
                pass
    
    def dijkstra(self, source, dest):
        # TODO
        q = queue()
        pass
        
    
    def plot(self):
        _, ax = plt.subplots()
        ax.set_aspect("equal", "box")

        x, y = self.boundary_polygon.exterior.xy
        ax.fill(x, y, alpha=0.3, color="red")
        
        
        x_edge, y_edge = [], []
        for edge in self.visibility_edges:
            x_edge.append(edge[0])
            y_edge.append(edge[1])
        for p, q in self.visibility_edges:
            ax.plot([p.x, q.x], [p.y, q.y], color="blue", alpha=0.5)

        for p in self.boundary_polygon.exterior.coords:
            ax.plot(p[0], p[1], "ro")
        plt.show()
    
    
    def is_visible(self, n1: Node, n2: Node) -> bool:
        return (n1, n2) in self.edges
    
    def construct_graph():
        pass
        
    def visible_nodes(self, n: Node) -> list[Node]:
        pass

boundary = [
    Node(0, 0),
    Node(10, 0),
    Node(10, 10),
    Node(0, 10)
]

# Create some waypoints inside the boundary
waypoints = [
    Node(2, 2),
    Node(5, 5),
    Node(8, 3),
    Node(3, 8)
]

# Build and plot the visibility graph
pf = Pathfinding(boundary, waypoints)
pf.plot()