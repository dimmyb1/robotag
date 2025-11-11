#!/usr/bin/env python3
import heapq
from copy import deepcopy

class GraphTSP:
    def __init__(self, graph):
        self.graph = graph

    def dijkstra(self, start):
        """Compute shortest paths from start to all nodes."""
        dist = {node: float('inf') for node in self.graph}
        prev = {node: None for node in self.graph}
        dist[start] = 0
        queue = [(0, start)]
        while queue:
            d, node = heapq.heappop(queue)
            if d > dist[node]:
                continue
            for neighbor, w in self.graph[node].items():
                if dist[node] + w < dist[neighbor]:
                    dist[neighbor] = dist[node] + w
                    prev[neighbor] = node
                    heapq.heappush(queue, (dist[neighbor], neighbor))
        return dist, prev
    
    def reconstruct_path(self, prev, start, end):
        """Reconstruct full path from Dijkstra results."""
        path = []
        node = end
        while node is not None:
            path.append(node)
            node = prev[node]
        path.reverse()
        if path[0] == start:
            return path
        return []  # No valid path
    
    def nearest_neighbour_tsp(self, start, targets):
        """Generate route visiting only specified target nodes using graph paths."""
        unvisited = set(targets)
        if start in unvisited:
            unvisited.remove(start)

        route = [start]
        current = start

        while unvisited:
            distances, prev = self.dijkstra(current)
            reachable_targets = [x for x in unvisited if distances[x] < float('inf')]
            if not reachable_targets:
                raise ValueError(f"No reachable targets remaining from {current}")
            # pick the nearest unvisited node along graph
            next_node = min(unvisited, key=lambda x: distances[x])
            segment = self.reconstruct_path(prev, current, next_node)
            route += segment[1:]
            unvisited.remove(next_node)
            current = next_node

        if start in self.graph[current]:
            route.append(start)
        else:
            distances, prev = self.dijkstra(current)
            if distances[start] < float('inf'):
                segment = self.reconstruct_path(prev, current, start)
                route += segment[1:]
            else:
                raise ValueError(f"Cannot return to {start} from {current} — no valid path")

        return route

    def total_distance(self, route):
        """Sum distances along the graph using shortest paths between consecutive nodes."""
        total = 0
        for i in range(len(route)-1):
            start, end = route[i], route[i+1]
            distances, _ = self.dijkstra(start)
            total += distances[end]
        return total

    def two_opt(self, route):
        """Improve route using 2-opt swaps along graph paths."""
        improved = True
        best_route = deepcopy(route)
        best_distance = self.total_distance(best_route)

        while improved:
            improved = False
            for i in range(1, len(best_route) - 2):
                for j in range(i + 1, len(best_route) - 1):
                    new_route = self.swap_2opt(best_route, i, j)
                    new_distance = self.total_distance(new_route)
                    # If the route created by two opt does not exist it will return infinity and therefore would not be chosen
                    if new_distance < best_distance:
                        best_route = new_route
                        best_distance = new_distance
                        improved = True
        return best_route

    def swap_2opt(self, route, i, j):
        return route[:i] + route[i:j+1][::-1] + route[j+1:]
