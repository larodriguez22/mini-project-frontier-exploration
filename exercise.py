from collections import deque
import heapq


def detect_frontiers(occupancy_map, map_width, map_height):
    """
    Detects frontiers in the occupancy map.
    reference the Notes and HW for definition of frontier cells.

    Args:
        occupancy_map 2D Array: The robot's 2D map.
            - 0: Known free space
            - 1: Known occupied space
            - 0.5: Unknown space
        map_width (int): The width of the map.
        map_height (int): The height of the map.

    Returns:
        list of tuples: A list of (x, y) coordinates for each frontier cell.
    """
    """
       Hint:
          - The directions to check for neighbors are: (0,1), (0,-1), (1,0), (-1,0)
          - A frontier must be a known, open cell (value 0) that is adjacent to at least one unknown cell (value 0.5).
          - BEWARE: Be careful not to check outside the bounds of the map!
    """
    frontiers = []

    for y in range(map_height):
        for x in range(map_width):
            if occupancy_map[y][x] != 0:
                continue
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < map_width and 0 <= ny < map_height and occupancy_map[ny][nx] == 0.5:
                    frontiers.append((x, y))
                    break

    return frontiers

def calculate_frontier_centroids(frontiers, occupancy_map, map_width, map_height):
    """
    Groups adjacent frontier cells into clusters and calculates the centroid of each cluster.

    Args:
        frontiers (list of tuples): A list of (x, y) coordinates for each frontier cell.
        occupancy_map, map_width, map_height: Map details.

    Returns:
        list of tuples: A list of (x, y) coordinates representing the centroid of each frontier group.
    """
    centroids = []
    MAX_CLUSTER_SIZE = 10
    
    frontier_set = set(frontiers)
    visited = set()

    for start in frontiers:
        if start in visited:
            continue

        # BFS to get one connected component.
        component = []
        queue = deque([start])
        visited.add(start)
        while queue:
            x, y = queue.popleft()
            component.append((x, y))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (x + dx, y + dy)
                    if n in frontier_set and n not in visited:
                        visited.add(n)
                        queue.append(n)

        for k in range(0, len(component), MAX_CLUSTER_SIZE):
            segment = component[k:k + MAX_CLUSTER_SIZE]
            mean_x = sum(c[0] for c in segment) / len(segment)
            mean_y = sum(c[1] for c in segment) / len(segment)
            # Get closest frontier to the mean position of the component
            centroid = min(segment, key=lambda c: (c[0] - mean_x) ** 2 + (c[1] - mean_y) ** 2)
            centroids.append(centroid)

    return centroids

def find_closest_frontier(robot_pos, frontier_centroids, occupancy_map, map_width, map_height):
    """
    Finds the frontier centroid that is closest to the robot and reachable.
    This function should use a pathfinding algorithm like A* to find a path.

    Args:
        robot_pos (tuple): The (x, y) position of the robot.
        frontier_centroids (list of tuples): Centroids of frontier groups.
        occupancy_map: The robot's 2D map.
        map_width (int): The width of the map.
        map_height (int): The height of the map.

    Returns:
        tuple or None: A tuple containing (target_goal, path_to_goal)
                       - target_goal: The (x, y) coordinate of the chosen frontier goal.
                       - path_to_goal: A list of (x, y) coordinates from robot to goal.
                       Returns (None, []) if no frontiers are reachable.
    """
    best_target = None
    shortest_path = []

    start = tuple(robot_pos)
    targets = set(frontier_centroids)
    if not targets:
        return best_target, shortest_path

    came_from = {start: None}
    queue = deque([start])
    while queue:
        current = queue.popleft()
        if current in targets and current != start:
            best_target = current
            break
        x, y = current
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            n = (x + dx, y + dy)
            nx, ny = n
            if (0 <= nx < map_width and 0 <= ny < map_height
                    and n not in came_from and occupancy_map[ny][nx] == 0):
                came_from[n] = current
                queue.append(n)

    if best_target is None:
        return None, []

    node = best_target
    while node != start:
        shortest_path.append(node)
        node = came_from[node]
    shortest_path.reverse()

    return best_target, shortest_path
