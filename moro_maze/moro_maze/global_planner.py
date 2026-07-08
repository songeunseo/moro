from collections import deque
import math
import os

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetMap
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R


class GlobalPlanner(Node):
    """Build a grid graph from the map and publish a global path."""

    def __init__(self):
        super().__init__('global_planner')

        self.declare_parameter('map_service', '/map_server/map')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('path_topic', '/global_planner/path')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_period', 2.0)
        self.declare_parameter('start_x', float('nan'))
        self.declare_parameter('start_y', float('nan'))
        self.declare_parameter('goal_x', float('nan'))
        self.declare_parameter('goal_y', float('nan'))
        self.declare_parameter('exit_row', 22)
        self.declare_parameter('exit_col', 22)
        self.declare_parameter('final_yaw', 0.0)
        self.declare_parameter('candidate_rows', [4, 10, 16, 22])
        self.declare_parameter('candidate_cols', [4, 10, 16, 22])
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('allow_unknown', False)
        self.declare_parameter('obstacle_padding_cells', 0)

        self.frame_id = self.get_parameter('frame_id').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_service = self.get_parameter('map_service').value
        self.map_yaml = self.get_parameter('map_yaml').value
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.allow_unknown = bool(self.get_parameter('allow_unknown').value)
        self.obstacle_padding_cells = int(
            self.get_parameter('obstacle_padding_cells').value)
        self.exit_row = int(self.get_parameter('exit_row').value)
        self.exit_col = int(self.get_parameter('exit_col').value)
        self.final_yaw = float(self.get_parameter('final_yaw').value)
        self.candidate_rows = [
            int(v) for v in self.get_parameter('candidate_rows').value]
        self.candidate_cols = [
            int(v) for v in self.get_parameter('candidate_cols').value]

        self.map_msg = None
        self.map_future = None
        self.free_grid = None
        self.graph = None
        self.graph_nodes = None
        self.last_path = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.path_pub = self.create_publisher(
            Path, self.get_parameter('path_topic').value, 10)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(
            OccupancyGrid, self.get_parameter('map_topic').value, map_qos)
        self.map_client = self.create_client(GetMap, self.map_service)

        period = float(self.get_parameter('publish_period').value)
        self.timer = self.create_timer(period, self.plan_and_publish)
        self.get_logger().info('GlobalPlanner node started.')

    def plan_and_publish(self):
        if self.map_msg is not None:
            self.publish_map()

        if self.last_path is not None:
            self.publish_path(self.last_path)
            return

        if self.map_msg is None:
            self.map_msg = self.request_map()
            if self.map_msg is None:
                return
            self.publish_map()
            self.free_grid = self.build_free_grid(self.map_msg)
            self.graph_nodes, self.graph = self.create_sparse_graph()

        start = self.resolve_start()
        goal = self.resolve_goal()
        if start is None or goal is None:
            self.get_logger().error('Could not resolve start or goal cell.')
            return

        path_cells = self.bfs_graph(start, goal)
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
                if self.map_yaml:
                    self.get_logger().warn(
                        f'Map service {self.map_service} not available yet. '
                        f'Loading static map from {self.map_yaml}.')
                    return self.load_map_from_yaml(self.map_yaml)
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

    def load_map_from_yaml(self, yaml_path):
        config = self.read_simple_yaml(yaml_path)
        image_path = config.get('image')
        if not image_path:
            self.get_logger().error(f'Map yaml {yaml_path} has no image entry.')
            return None
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(yaml_path), image_path)

        resolution = float(config.get('resolution', 1.0))
        origin = [
            float(v) for v in config.get('origin', '0, 0, 0')
            .strip('[]')
            .split(',')]
        negate = int(config.get('negate', 0))
        occupied_thresh = float(config.get('occupied_thresh', 0.65))
        free_thresh = float(config.get('free_thresh', 0.196))

        pixels, width, height = self.read_pgm(image_path)
        # PGM rows are stored top-to-bottom, while OccupancyGrid data starts at
        # the map origin in the lower-left corner.
        pixels = np.flipud(pixels)
        if negate:
            occ = pixels.astype(np.float32) / 255.0
        else:
            occ = (255.0 - pixels.astype(np.float32)) / 255.0

        data = np.full((height, width), -1, dtype=np.int8)
        data[occ > occupied_thresh] = 100
        data[occ < free_thresh] = 0

        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = origin[0]
        msg.info.origin.position.y = origin[1]
        msg.info.origin.orientation.w = 1.0
        msg.data = data.reshape(-1).astype(int).tolist()
        self.get_logger().info(
            f'Loaded static map from {image_path}: {width}x{height}, res={resolution}.')
        return msg

    @staticmethod
    def read_simple_yaml(path):
        config = {}
        with open(path, 'r', encoding='utf-8') as file:
            for line in file:
                line = line.split('#', 1)[0].strip()
                if not line or ':' not in line:
                    continue
                key, value = line.split(':', 1)
                config[key.strip()] = value.strip()
        return config

    @staticmethod
    def read_pgm(path):
        with open(path, 'rb') as file:
            magic = file.readline().strip()
            if magic != b'P5':
                raise ValueError(f'Unsupported PGM format in {path}: {magic!r}')

            line = file.readline()
            while line.startswith(b'#'):
                line = file.readline()
            width, height = [int(v) for v in line.split()]

            line = file.readline()
            while line.startswith(b'#'):
                line = file.readline()
            max_value = int(line)
            if max_value != 255:
                raise ValueError(f'Unsupported PGM max value in {path}: {max_value}')

            pixels = np.frombuffer(file.read(), dtype=np.uint8)
        return pixels.reshape((height, width)), width, height

    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_msg)

    def localise_robot(self):
        errors = []
        for frame in (self.frame_id, self.odom_frame):
            try:
                trans = self.tf_buffer.lookup_transform(
                    frame, self.base_frame, Time(),
                    timeout=Duration(seconds=0.2))
                break
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as exc:
                errors.append(f'{frame}->{self.base_frame}: {exc}')
        else:
            raise RuntimeError(f'TF lookup failed: {"; ".join(errors)}')

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

    def create_sparse_graph(self):
        nodes = []
        rows, cols = self.free_grid.shape

        for row in self.candidate_rows:
            for col in self.candidate_cols:
                if 0 <= row < rows and 0 <= col < cols and self.free_grid[row, col]:
                    nodes.append((row, col))

        node_set = set(nodes)
        graph = {node: set() for node in nodes}

        for row, col in nodes:
            for next_col in self.candidate_cols:
                if next_col <= col or (row, next_col) not in node_set:
                    continue
                if self.path_is_free(row, col, row, next_col):
                    graph[(row, col)].add((row, next_col))
                    graph[(row, next_col)].add((row, col))
                    break

            for next_row in self.candidate_rows:
                if next_row <= row or (next_row, col) not in node_set:
                    continue
                if self.path_is_free(row, col, next_row, col):
                    graph[(row, col)].add((next_row, col))
                    graph[(next_row, col)].add((row, col))
                    break

        return nodes, graph

    def path_is_free(self, row1, col1, row2, col2):
        if row1 == row2:
            start = min(col1, col2)
            end = max(col1, col2)
            return all(self.is_free(row1, col) for col in range(start, end + 1))

        if col1 == col2:
            start = min(row1, row2)
            end = max(row1, row2)
            return all(self.is_free(row, col1) for row in range(start, end + 1))

        return False

    def is_free(self, row, col):
        rows, cols = self.free_grid.shape
        return 0 <= row < rows and 0 <= col < cols and self.free_grid[row, col]

    def resolve_goal(self):
        goal_x = float(self.get_parameter('goal_x').value)
        goal_y = float(self.get_parameter('goal_y').value)
        if math.isfinite(goal_x) and math.isfinite(goal_y):
            return self.nearest_graph_node(goal_x, goal_y)

        if (self.exit_row, self.exit_col) in self.graph:
            return (self.exit_row, self.exit_col)

        x, y = self.map_to_world(self.exit_row, self.exit_col)
        return self.nearest_graph_node(x, y)

    def resolve_start(self):
        start_x = float(self.get_parameter('start_x').value)
        start_y = float(self.get_parameter('start_y').value)
        if math.isfinite(start_x) and math.isfinite(start_y):
            return self.nearest_graph_node(start_x, start_y)

        try:
            robot_pose = self.localise_robot()
        except RuntimeError as exc:
            self.get_logger().warn(str(exc))
            return None

        return self.nearest_graph_node(robot_pose[0], robot_pose[1])

    def nearest_graph_node(self, x, y):
        best = None
        best_dist = float('inf')
        for node in self.graph_nodes:
            node_x, node_y = self.map_to_world(*node)
            dist = (node_x - x) ** 2 + (node_y - y) ** 2
            if dist < best_dist:
                best = node
                best_dist = dist
        return best

    def bfs_graph(self, start, goal):
        queue = deque([start])
        parent = {start: None}

        while queue:
            current = queue.popleft()
            if current == goal:
                break

            for child in self.graph.get(current, []):
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

    def cells_to_poses(self, path_cells):
        points = [self.map_to_world(row, col) for row, col in path_cells]
        poses = []
        for idx, (x, y) in enumerate(points):
            if idx == len(points) - 1:
                theta = self.final_yaw
            else:
                next_x, next_y = points[idx + 1]
                theta = math.atan2(next_y - y, next_x - x)
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
