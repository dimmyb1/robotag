#!/usr/bin/env python3
import heapq
import itertools
from copy import deepcopy

class BruteForceTSP:
    def __init__(self, graph):
        self.graph = graph

    def dijkstra(self, start):
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
        path = []
        node = end
        while node is not None:
            path.append(node)
            node = prev[node]
        path.reverse()
        if path[0] == start:
            return path
        return []

    def path_and_distance(self, start, end):
        dist, prev = self.dijkstra(start)
        if dist[end] == float('inf'):
            return [], float('inf')
        path = self.reconstruct_path(prev, start, end)
        return path, dist[end]

    def total_distance(self, route):
        total = 0
        for i in range(len(route) - 1):
            _, dist = self.path_and_distance(route[i], route[i + 1])
            total += dist
        return total

    def brute_force_tsp(self, start, targets):
        nodes_to_visit = [t for t in targets if t != start]
        best_route = None
        best_distance = float('inf')

        # Try every possible visiting order
        for perm in itertools.permutations(nodes_to_visit):
            current = start
            route = [start]
            total = 0
            valid = True

            # Build the path between consecutive nodes
            for next_node in perm:
                segment, dist = self.path_and_distance(current, next_node)
                if dist == float('inf'):
                    valid = False
                    break
                route += segment[1:]
                total += dist
                current = next_node

            # Add return to start
            if valid:
                segment, dist = self.path_and_distance(current, start)
                if dist < float('inf'):
                    route += segment[1:]
                    total += dist
                    if total < best_distance:
                        best_distance = total
                        best_route = deepcopy(route)

        if best_route is None:
            raise ValueError("No valid TSP route found.")

        return best_route
