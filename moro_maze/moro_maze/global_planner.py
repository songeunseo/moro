import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

import numpy as np
from collections import deque


class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('global_planner')
    

        self.recMap = self.get_map()
        self.grid = np.array(self.recMap.data).reshape(
            self.recMap.info.height,
            self.recMap.info.width
        )

        self.nodes, self.edges = self.create_sparse_graph(self.grid)

        self.global_path = None
        self.path_pub = self.create_publisher(Path, '/global_planner/path', 10)
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10)
        self.timer = self.create_timer(1.0, self.publish_path)

        self.get_logger().info(
            "Waiting for RViz 2D Pose Estimate on /initialpose")

    def initial_pose_callback(self, msg):
        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y

        exit_x = 4.5
        exit_y = 4.0

        start = self.get_nearest_node(robot_x, robot_y)
        goal = self.get_nearest_node(exit_x, exit_y)

        self.get_logger().info(f"Start node: {start}")
        self.get_logger().info(f"Goal node: {goal}")

        discovered_nodes = self.bfs(start, goal)
        path = self.reconstruct_path(discovered_nodes, start, goal)

        self.get_logger().info(f"Found path: {path}")

        global_path = self.make_global_path(path)

# Add final waypoint to drive closer to the maze edge
        final_x = 4.5
        final_y = 4.0
        
        last_x = global_path[-1][0]
        last_y = global_path[-1][1]
        
        theta = np.arctan2(final_y - last_y, final_x - last_x)
        
        # Make the previous final node face toward the exit
        global_path[-1][2] = theta
        
        # Add maze-edge goal
        global_path.append([final_x, final_y, theta])
        
        self.global_path = global_path

        self.get_logger().info(str(global_path))
        self.get_logger().info("Publishing path on /global_planner/path")
        
    def get_map(self, timeout_sec=5.0) -> OccupancyGrid:
        client = self.create_client(GetMap, '/map_server/map')

        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("/map_server/map service not available")

        request = GetMap.Request()
        future = client.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if not future.done() or future.result() is None:
            raise RuntimeError("GetMap call failed")

        return future.result().map

    def is_free(self, row, col):
        h, w = self.grid.shape

        if row < 0 or row >= h or col < 0 or col >= w:
            return False

        return self.grid[row, col] == 0

    def to_node_name(self, row, col):
        return f"{row}.{col}"

    def from_node_name(self, name):
        row, col = name.split(".")
        row = int(row)
        col = int(col)

        resolution = self.recMap.info.resolution
        origin_x = self.recMap.info.origin.position.x
        origin_y = self.recMap.info.origin.position.y

        x = origin_x + (col + 0.5) * resolution
        y = origin_y + (row + 0.5) * resolution

        return np.array([x, y])

    def path_is_free(self, row1, col1, row2, col2):
        if row1 == row2:
            start = min(col1, col2)
            end = max(col1, col2)

            for col in range(start, end + 1):
                if not self.is_free(row1, col):
                    return False

            return True

        elif col1 == col2:
            start = min(row1, row2)
            end = max(row1, row2)

            for row in range(start, end + 1):
                if not self.is_free(row, col1):
                    return False

            return True

        else:
            return False

    def create_sparse_graph(self, grid):
        candidate_rows = [4, 10, 16, 22]
        candidate_cols = [4, 10, 16, 22]

        nodes = []

        for row in candidate_rows:
            for col in candidate_cols:
                if self.is_free(row, col):
                    nodes.append((row, col))

        node_set = set(nodes)
        edges = []

        for row, col in nodes:
            for next_col in candidate_cols:
                if next_col <= col:
                    continue

                if (row, next_col) not in node_set:
                    continue

                if self.path_is_free(row, col, row, next_col):
                    edges.append({
                        "parent": self.to_node_name(row, col),
                        "child": self.to_node_name(row, next_col),
                        "cost": abs(next_col - col)
                    })
                    break

        for row, col in nodes:
            for next_row in candidate_rows:
                if next_row <= row:
                    continue

                if (next_row, col) not in node_set:
                    continue

                if self.path_is_free(row, col, next_row, col):
                    edges.append({
                        "parent": self.to_node_name(row, col),
                        "child": self.to_node_name(next_row, col),
                        "cost": abs(next_row - row)
                    })
                    break

        return nodes, edges

    def get_nearest_node(self, robot_x, robot_y):
        min_dist = float("inf")
        nearest_node = None

        for row, col in self.nodes:
            node_name = self.to_node_name(row, col)
            node_pos = self.from_node_name(node_name)

            x = node_pos[0]
            y = node_pos[1]

            dist = ((x - robot_x) ** 2 + (y - robot_y) ** 2) ** 0.5

            if dist < min_dist:
                min_dist = dist
                nearest_node = node_name

        return nearest_node

    def bfs(self, start, goal):
        queue = deque([start])
        discovered_nodes = {start: None}

        while queue:
            current = queue.popleft()

            if current == goal:
                break

            for edge in self.edges:
                parent = edge["parent"]
                child = edge["child"]

                neighbors = []

                if parent == current:
                    neighbors.append(child)

                if child == current:
                    neighbors.append(parent)

                for neighbor in neighbors:
                    if neighbor not in discovered_nodes:
                        discovered_nodes[neighbor] = current
                        queue.append(neighbor)

        return discovered_nodes

    def reconstruct_path(self, discovered_nodes, start, goal):
        if goal not in discovered_nodes:
            self.get_logger().warn("No path found")
            return []

        path = []
        current = goal

        while current is not None:
            path.append(current)
            current = discovered_nodes[current]

        path.reverse()
        return path

    def make_global_path(self, path):
        global_path = []

        for i, node in enumerate(path):
            x, y = self.from_node_name(node)

            if i < len(path) - 1:
                next_x, next_y = self.from_node_name(path[i + 1])
                theta = np.arctan2(next_y - y, next_x - x)
            else:
                theta = global_path[-1][2] if len(global_path) > 0 else 0.0

            global_path.append([float(x), float(y), float(theta)])

        return global_path

    def publish_path(self):
        if self.global_path is None:
            return

        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
    
        for x, y, theta in self.global_path:
            pose = PoseStamped()
            pose.header = msg.header
    
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
    
            pose.pose.orientation.z = float(np.sin(theta / 2.0))
            pose.pose.orientation.w = float(np.cos(theta / 2.0))
    
            msg.poses.append(pose)
    
        self.path_pub.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)

    node = GlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
