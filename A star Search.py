import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
import heapq

class AStarPlanner:
    def __init__(self, grid):
        self.grid = grid
        self.grid_height = grid.shape[0]
        self.grid_width = grid.shape[1]

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance

    def get_neighbors(self, node):
        neighbors = [
            (node[0] - 1, node[1]), (node[0] + 1, node[1]),
            (node[0], node[1] - 1), (node[0], node[1] + 1)
        ]
        valid_neighbors = [
            n for n in neighbors if 0 <= n[0] < self.grid_height and 0 <= n[1] < self.grid_width
            and self.grid[n[0], n[1]] == 0  # Not an obstacle
        ]
        return valid_neighbors

    def plan(self, start, goal):
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_list:
            _, current = heapq.heappop(open_list)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))

        return None  # No path found

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path


def occupancy_grid_callback(data):
    # Convert OccupancyGrid to a numpy array
    width, height = data.info.width, data.info.height
    grid = np.array(data.data).reshape((height, width))
    grid = (grid > 50).astype(int)  # Thresholding: 1 = obstacle, 0 = free space

    start = (5, 5)  # Replace with actual start position
    goal = (25, 25)  # Replace with actual goal position

    planner = AStarPlanner(grid)
    path = planner.plan(start, goal)

    if path:
        rospy.loginfo(f"Path found: {path}")
    else:
        rospy.loginfo("No path found.")

if __name__ == "__main__":
    rospy.init_node('astar_planner', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, occupancy_grid_callback)
    rospy.spin()
