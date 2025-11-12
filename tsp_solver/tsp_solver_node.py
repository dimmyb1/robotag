#!/usr/bin/env python3
import heapq
from copy import deepcopy

class GraphTSP:
    def __init__(self, graph):
        self.graph = graph

    def dijkstra(self, start):
        # mark each distance as infinity
        # dictionary of shortest distances to every node.
        dist = {node: float('inf') for node in self.graph}
        # dictionary to reconstruct the path.
        prev = {node: None for node in self.graph}
        # Start with a 0 distance
        dist[start] = 0
        queue = [(0, start)]
        while queue:
            # use a priority queu to always explore the smallest distance
            d, node = heapq.heappop(queue)
            if d > dist[node]:
                continue
            for neighbor, w in self.graph[node].items():
                # for each neighbour check if going through the current node is less
                if dist[node] + w < dist[neighbor]:
                    dist[neighbor] = dist[node] + w
                    # Keep track of each node’s predecessor (prev) so we can rebuild the actual route later.
                    prev[neighbor] = node
                    heapq.heappush(queue, (dist[neighbor], neighbor))
        return dist, prev
    
    def reconstruct_path(self, prev, start, end):
        path = []
        node = end
        # rebuild path from start to end
        while node is not None:
            path.append(node)
            node = prev[node]
        path.reverse()
        if path[0] == start:
            return path
        return []  # No valid path
    
    def nearest_neighbour_tsp(self, start, targets):
        # The brain of the robot
        unvisited = set(targets)
        if start in unvisited:
            unvisited.remove(start)

        route = [start]
        current = start

        # while there are still unvisited houses to deliver
        while unvisited:
            # run djikstras from this node
            distances, prev = self.dijkstra(current)
            # find all the reachabled targets
            reachable_targets = [x for x in unvisited if distances[x] < float('inf')]
            if not reachable_targets:
                raise ValueError(f"No reachable targets remaining from {current}")
            # pick the nearest unvisited node along graph
            next_node = min(unvisited, key=lambda x: distances[x])
            # add that path to the overall route.
            segment = self.reconstruct_path(prev, current, next_node)
            route += segment[1:]
            unvisited.remove(next_node)
            current = next_node

        # return to the PostOffice
        # find djikstras path
        distances, prev = self.dijkstra(current)
        # check in case there is not a path to raise an error if so
        if distances[start] < float('inf'):
            segment = self.reconstruct_path(prev, current, start)
            route += segment[1:]
        else:
            raise ValueError(f"Cannot return to {start} from {current} — no valid path")

        return route

    def total_distance(self, route):
        total = 0
        for i in range(len(route)-1):
            # sum the path
            start, end = route[i], route[i+1]
            distances, _ = self.dijkstra(start)
            total += distances[end]
        return total

    def two_opt(self, route):

        improved = True
        best_route = deepcopy(route)
        best_distance = self.total_distance(best_route)

        while improved:
            improved = False
            # look at every possible pair of edges, swap them, and see if the route becomes shorter
            # repeat only until there is improvement
            for i in range(1, len(best_route) - 2):
                for j in range(i + 1, len(best_route) - 1):
                    new_route = self.swap_2opt(best_route, i, j)
                    new_distance = self.total_distance(new_route)
                    # if the route created by two opt does not exist it will return infinity and therefore would not be chosen
                    if new_distance < best_distance:
                        best_route = new_route
                        best_distance = new_distance
                        improved = True
        return best_route

    def swap_2opt(self, route, i, j):
        # this helper function reverses the section between indices i and j in the route.
        return route[:i] + route[i:j+1][::-1] + route[j+1:]
