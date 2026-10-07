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

    # Your code goes here

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

    # Your code goes here

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

    # Your code goes here

    return best_target, shortest_path
