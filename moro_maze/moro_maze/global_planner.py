from collections import deque
import math

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetMap
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R


class GlobalPlanner(Node):
    """Build a grid graph from the map and publish a global path."""

    def __init__(self):
        super().__init__('global_planner')

        self.declare_parameter('map_service', '/map_server/map')
        self.declare_parameter('path_topic', '/global_planner/path')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_period', 2.0)
        self.declare_parameter('goal_x', float('nan'))
        self.declare_parameter('goal_y', float('nan'))
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('allow_unknown', False)
        self.declare_parameter('obstacle_padding_cells', 0)
        self.declare_parameter('use_diagonal_motion', False)

        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_service = self.get_parameter('map_service').value
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.allow_unknown = bool(self.get_parameter('allow_unknown').value)
        self.obstacle_padding_cells = int(
            self.get_parameter('obstacle_padding_cells').value)
        self.use_diagonal_motion = bool(
            self.get_parameter('use_diagonal_motion').value)

        self.map_msg = None
        self.map_future = None
        self.free_grid = None
        self.last_path = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.path_pub = self.create_publisher(
            Path, self.get_parameter('path_topic').value, 10)
        self.map_client = self.create_client(GetMap, self.map_service)

        period = float(self.get_parameter('publish_period').value)
        self.timer = self.create_timer(period, self.plan_and_publish)
        self.get_logger().info('GlobalPlanner node started.')

    def plan_and_publish(self):
        if self.map_msg is None:
            self.map_msg = self.request_map()
            if self.map_msg is None:
                return
            self.free_grid = self.build_free_grid(self.map_msg)

        try:
            robot_pose = self.localise_robot()
        except RuntimeError as exc:
            self.get_logger().warn(str(exc))
            if self.last_path is not None:
                self.publish_path(self.last_path)
            return

        start = self.nearest_free_cell(robot_pose[0], robot_pose[1])
        goal = self.resolve_goal(start)
        if start is None or goal is None:
            self.get_logger().error('Could not resolve start or goal cell.')
            return

        path_cells = self.bfs(start, goal)
        if not path_cells:
            self.get_logger().error(
                f'No path found from {start} to {goal}.')
            return

        path_poses = self.cells_to_poses(path_cells)
        self.last_path = path_poses
        self.publish_path(path_poses)
        self.get_logger().info(
            f'Published global path with {len(path_poses)} poses: {start} -> {goal}')

    def request_map(self):
        if self.map_future is None:
            if not self.map_client.wait_for_service(timeout_sec=0.1):
                self.get_logger().warn(
                    f'Map service {self.map_service} not available yet.')
                return None
            self.map_future = self.map_client.call_async(GetMap.Request())
            self.get_logger().info(f'Requested map from {self.map_service}.')
            return None

        if not self.map_future.done():
            return None

        if self.map_future.result() is None:
            self.get_logger().warn('GetMap call failed.')
            self.map_future = None
            return None

        return self.map_future.result().map

    def localise_robot(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.frame_id, self.base_frame, Time(),
                timeout=Duration(seconds=0.2))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            raise RuntimeError(f'TF lookup failed: {exc}') from exc

        theta = R.from_quat([
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w]).as_euler('xyz')[2]
        return np.array([
            trans.transform.translation.x,
            trans.transform.translation.y,
            theta])

    def build_free_grid(self, map_msg: OccupancyGrid):
        height = map_msg.info.height
        width = map_msg.info.width
        data = np.array(map_msg.data, dtype=np.int16).reshape((height, width))

        if self.allow_unknown:
            free = data < self.occupied_threshold
        else:
            free = (data >= 0) & (data < self.occupied_threshold)

        for _ in range(max(0, self.obstacle_padding_cells)):
            occupied = ~free
            padded = occupied.copy()
            padded[1:, :] |= occupied[:-1, :]
            padded[:-1, :] |= occupied[1:, :]
            padded[:, 1:] |= occupied[:, :-1]
            padded[:, :-1] |= occupied[:, 1:]
            free = ~padded

        return free

    def resolve_goal(self, start):
        goal_x = float(self.get_parameter('goal_x').value)
        goal_y = float(self.get_parameter('goal_y').value)
        if math.isfinite(goal_x) and math.isfinite(goal_y):
            return self.nearest_free_cell(goal_x, goal_y)
        return self.farthest_boundary_cell(start)

    def farthest_boundary_cell(self, start):
        if start is None:
            return None
        cells = []
        rows, cols = self.free_grid.shape
        for row in range(rows):
            for col in range(cols):
                if not self.free_grid[row, col]:
                    continue
                if row in (0, rows - 1) or col in (0, cols - 1):
                    cells.append((row, col))
        if not cells:
            return None
        return max(cells, key=lambda c: (c[0] - start[0]) ** 2 + (c[1] - start[1]) ** 2)

    def nearest_free_cell(self, x, y):
        row, col = self.world_to_map(x, y)
        rows, cols = self.free_grid.shape
        if 0 <= row < rows and 0 <= col < cols and self.free_grid[row, col]:
            return (row, col)

        best = None
        best_dist = float('inf')
        free_rows, free_cols = np.where(self.free_grid)
        for free_row, free_col in zip(free_rows, free_cols):
            dist = (free_row - row) ** 2 + (free_col - col) ** 2
            if dist < best_dist:
                best = (int(free_row), int(free_col))
                best_dist = dist
        return best

    def bfs(self, start, goal):
        queue = deque([start])
        parent = {start: None}

        while queue:
            current = queue.popleft()
            if current == goal:
                break

            for child in self.neighbours(current):
                if child in parent:
                    continue
                parent[child] = current
                queue.append(child)

        if goal not in parent:
            return []

        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = parent[current]
        path.reverse()
        return path

    def neighbours(self, cell):
        row, col = cell
        moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        if self.use_diagonal_motion:
            moves += [(-1, -1), (-1, 1), (1, -1), (1, 1)]

        rows, cols = self.free_grid.shape
        for d_row, d_col in moves:
            next_row = row + d_row
            next_col = col + d_col
            if not (0 <= next_row < rows and 0 <= next_col < cols):
                continue
            if self.free_grid[next_row, next_col]:
                yield (next_row, next_col)

    def cells_to_poses(self, path_cells):
        points = [self.map_to_world(row, col) for row, col in path_cells]
        poses = []
        for idx, (x, y) in enumerate(points):
            if idx < len(points) - 1:
                next_x, next_y = points[idx + 1]
                theta = math.atan2(next_y - y, next_x - x)
            elif poses:
                theta = poses[-1][2]
            else:
                theta = 0.0
            poses.append((x, y, theta))
        return poses

    def publish_path(self, path_poses):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        for x, y, theta in path_poses:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            qx, qy, qz, qw = R.from_euler('z', theta).as_quat()
            pose.pose.orientation.x = float(qx)
            pose.pose.orientation.y = float(qy)
            pose.pose.orientation.z = float(qz)
            pose.pose.orientation.w = float(qw)
            msg.poses.append(pose)

        self.path_pub.publish(msg)

    def world_to_map(self, x, y):
        info = self.map_msg.info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        return row, col

    def map_to_world(self, row, col):
        info = self.map_msg.info
        x = info.origin.position.x + (col + 0.5) * info.resolution
        y = info.origin.position.y + (row + 0.5) * info.resolution
        return x, y


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
