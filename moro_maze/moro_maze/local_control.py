import copy

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R


class PT2Block:
    """Discrete PT2 Block (Tustin approximation) - notebook과 동일"""

    def __init__(self, T=0, D=0, kp=1, ts=0, bufferLength=3) -> None:
        self.k1 = self.k2 = self.k3 = self.k4 = self.k5 = self.k6 = 0
        self.e = [0 for _ in range(bufferLength)]
        self.y = [0 for _ in range(bufferLength)]
        if ts != 0:
            self.setConstants(T, D, kp, ts)

    def setConstants(self, T, D, kp, ts) -> None:
        self.k1 = 4 * T ** 2 + 4 * D * T * ts + ts ** 2
        self.k2 = 2 * ts ** 2 - 8 * T ** 2
        self.k3 = 4 * T ** 2 - 4 * D * T * ts + ts ** 2
        self.k4 = kp * ts ** 2
        self.k5 = 2 * kp * ts ** 2
        self.k6 = kp * ts ** 2

    def update(self, e) -> float:
        self.e = [e] + self.e[:len(self.e) - 1]
        self.y = [0] + self.y[:len(self.y) - 1]
        e, y = self.e, self.y
        y[0] = (e[0] * self.k4 + e[1] * self.k5 + e[2] * self.k6
                - y[1] * self.k2 - y[2] * self.k3) / self.k1
        return y[0]


class LocalController(Node):

    def __init__(self):
        super().__init__('local_controller')

        # ---- Parameter ----
        self.ts = 0.5              # sampling time [sec] -> 2Hz control loop
        self.horizon = 10          # lookahead steps -> 5 sec lookahead
        self.goal_tolerance = 0.3  # [m]
        self.declare_parameter('cmd_vel_stamped', True)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('pose_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.cmd_vel_stamped = bool(self.get_parameter('cmd_vel_stamped').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.pose_frame = str(self.get_parameter('pose_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        # ---- State ----
        self.global_path = None       # list of [x, y, theta]
        self.current_goal_id = 0
        self.last_control = np.array([0.0, 0.0])
        self.robot_model_pt2 = PT2Block(ts=self.ts, T=0.05, D=0.8)

        # ---- TF ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Sub ----
        self.path_sub = self.create_subscription(
            Path, '/global_planner/path', self.path_callback, 10)

        # ---- Pub ----
        cmd_type = TwistStamped if self.cmd_vel_stamped else Twist
        self.cmd_pub = self.create_publisher(cmd_type, self.cmd_vel_topic, 10)
        self.trajectory_pub = self.create_publisher(Path, '/local_planner/trajectory', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/local_planner/goal', 10)

        # ---- Timer (control loop) ----
        self.timer = self.create_timer(self.ts, self.control_loop)

        self.get_logger().info(
            f'LocalController node started. Publishing '
            f'{"TwistStamped" if self.cmd_vel_stamped else "Twist"} on {self.cmd_vel_topic}.')

    # -------------------------------------------------
    # callback
    # -------------------------------------------------
    def path_callback(self, msg: Path):
        """Node 1이 발행하는 nav_msgs/Path -> [x, y, theta] 리스트로 변환"""
        path = []
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            q = pose_stamped.pose.orientation
            theta = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2]
            path.append([x, y, theta])

        if self.same_path(path):
            return

        self.global_path = path
        self.current_goal_id = 0
        self.get_logger().info(f'Received global path with {len(path)} waypoints.')

    def same_path(self, path):
        if self.global_path is None or len(path) != len(self.global_path):
            return False
        return np.allclose(np.array(path), np.array(self.global_path), atol=1e-3)

    def control_loop(self):
        if self.global_path is None:
            return  # Node 1 경로 아직 안 옴 -> 대기

        if self.current_goal_id >= len(self.global_path):
            self.pub_cmd([0.0, 0.0])
            return  # 이미 도착, 정지 유지

        # 1. 로봇 위치 파악
        try:
            robotpose = self.localise_robot()
        except RuntimeError as e:
            self.get_logger().warn(str(e))
            return

        # 2. 현재 목표
        goal = self.global_path[self.current_goal_id]

        # 3. 목표를 로봇 기준 좌표계로 변환
        T_robot = self.pose2tf_mat(robotpose)
        T_goal = self.pose2tf_mat(goal)
        goalpose = self.tf_mat2pose(np.linalg.inv(T_robot) @ T_goal)

        # 4. 제어 신호 생성
        controls = self.generate_controls(self.last_control)

        # 5. forward simulation으로 각 제어 평가
        costs, trajectories = self.evaluate_controls(controls, goalpose)

        # 6. 최적 제어 선택
        idx = np.argmin(costs)
        self.last_control = controls[idx]
        self.robot_model_pt2.update(self.last_control[0])
        dist = float(np.hypot(goalpose[0], goalpose[1]))

        # 7. 퍼블리시
        self.pub_cmd(controls[idx])
        self.get_logger().info(
            f'cmd v={controls[idx][0]:.3f}, w={controls[idx][1]:.3f}, '
            f'goal_dist={dist:.3f}, waypoint={self.current_goal_id}/{len(self.global_path)}',
            throttle_duration_sec=1.0)
        self.pub_trajectory(trajectories[idx])
        self.pub_goal(goalpose)

        # 8. 목표 도달 판정 -> 다음 waypoint로
        if dist < self.goal_tolerance:
            self.current_goal_id += 1
            self.get_logger().info(f'Reached waypoint {self.current_goal_id - 1}, moving to next.')
            if self.current_goal_id >= len(self.global_path):
                self.pub_cmd([0.0, 0.0])
                self.get_logger().info('Final goal reached. Stopping.')

    # -------------------------------------------------
    # 핵심 알고리즘 (notebook 함수 그대로 이식)
    # -------------------------------------------------
    def localise_robot(self) -> np.ndarray:
        errors = []
        for frame in (self.pose_frame, self.odom_frame):
            try:
                trans = self.tf_buffer.lookup_transform(
                    frame, self.base_frame, Time(), timeout=Duration(seconds=0.2))
                break
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                errors.append(f'{frame}->{self.base_frame}: {e}')
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

    @staticmethod
    def pose2tf_mat(pose):
        x, y, theta = pose
        return np.array([
            [np.cos(theta), -np.sin(theta), x],
            [np.sin(theta), np.cos(theta), y],
            [0, 0, 1]
        ])

    @staticmethod
    def tf_mat2pose(T):
        x = T[0, 2]
        y = T[1, 2]
        theta = np.arctan2(T[1, 0], T[0, 0])
        return np.array([x, y, theta])

    @staticmethod
    def generate_controls(last_control):
        last_control = np.array(last_control)
        v_min, v_max, v_step = -0.08, 0.12, 0.02
        v_delta = 0.06
        vt = np.arange(
            max(v_min, last_control[0] - v_delta),
            min(v_max, last_control[0] + v_delta) + v_step / 2,
            v_step)

        w_min, w_max, w_step = -1.4, 1.4, 0.05
        w_delta = 0.7
        wt = np.arange(
            max(w_min, last_control[1] - w_delta),
            min(w_max, last_control[1] + w_delta) + w_step / 2,
            w_step)

        return np.array([[v, w] for w in wt for v in vt])

    @staticmethod
    def forward_kinematics(control, last_pose, dt, dtype=np.float64):
        if not isinstance(last_pose, np.ndarray):
            last_pose = np.array(last_pose, dtype=dtype)
        if not isinstance(control, np.ndarray):
            control = np.array(control)

        vt, wt = control
        if wt == 0:
            wt = np.finfo(dtype).tiny

        vtwt = vt / wt
        _, _, theta = last_pose

        return last_pose + np.array([
            -vtwt * np.sin(theta) + vtwt * np.sin(theta + (wt * dt)),
            vtwt * np.cos(theta) - vtwt * np.cos(theta + (wt * dt)),
            wt * dt
        ], dtype=dtype)

    @staticmethod
    def cost_fn(pose, goalpose, control):
        e = np.abs(pose - goalpose)
        e[2] = abs(np.arctan2(np.sin(pose[2] - goalpose[2]),
                              np.cos(pose[2] - goalpose[2])))

        u = np.abs(control)

        Q = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 0.5]
        ])
        Rm = np.array([
            [0.03, 0],
            [0, 0.03]
        ])

        return e.T @ (Q @ e) + u.T @ (Rm @ u)

    def evaluate_controls(self, controls, goalpose):
        costs = np.zeros(len(controls), dtype=float)
        trajectories = [[] for _ in controls]

        for ctrl_idx, control in enumerate(controls):
            forward_sim_pt2 = copy.deepcopy(self.robot_model_pt2)
            forwardpose = np.array([0.0, 0.0, 0.0])

            for _ in range(self.horizon):
                v_t, w_t = control
                v_t_dynamic = forward_sim_pt2.update(v_t)
                control_dyn = [v_t_dynamic, w_t]
                forwardpose = self.forward_kinematics(control_dyn, forwardpose, self.ts)
                costs[ctrl_idx] += self.cost_fn(forwardpose, goalpose, control)
                trajectories[ctrl_idx].append(forwardpose.tolist())

        return costs, trajectories

    # -------------------------------------------------
    # Publisher
    # -------------------------------------------------
    def pub_cmd(self, control):
        if self.cmd_vel_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = float(control[0])
            msg.twist.angular.z = float(control[1])
        else:
            msg = Twist()
            msg.linear.x = float(control[0])
            msg.angular.z = float(control[1])
        self.cmd_pub.publish(msg)

    def pub_trajectory(self, trajectory):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        for pose in trajectory:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = float(pose[0])
            p.pose.position.y = float(pose[1])
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self.trajectory_pub.publish(msg)

    def pub_goal(self, goalpose):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = float(goalpose[0])
        msg.pose.position.y = float(goalpose[1])
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
