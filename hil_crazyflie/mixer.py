import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64, Float64MultiArray


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def quat_to_euler(x, y, z, w):
    # roll
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch
    sinp = 2.0 * (w * y - z * x)
    sinp = clamp(sinp, -1.0, 1.0)
    pitch = math.asin(sinp)

    # yaw (not used)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def angle_diff(a, b):
    """Smallest signed difference a-b, wrapped to [-pi, pi]."""
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class PoseStabilizerMixer(Node):
    def __init__(self):
        super().__init__('pose_stabilizer_mixer')

        # ---- Inputs ----
        self.pose_topic = '/world/quadcopter/pose/info'
        self.pose_index = 1  # your quad is poses[1]
        self.thrust_topic = 'thrust_cmd'  # from your altitude PD

        # ---- Outputs ----
        self.motor_topic = '/motor_cmds'  # goes to MotorCmdsToActuators

        # ---- Gains (start here) ----
        # Stabilize to level (roll=0, pitch=0)
        self.roll_sp = 0.0
        self.pitch_sp = 0.0

        # PD on angle + rate damping (angle rad, rate rad/s)
        self.Kp_roll = 0.10
        self.Kd_roll = 0.075
        self.Kp_pitch = 0.10
        self.Kd_pitch = 0.075

        # limit stabilization influence (normalized motor delta)
        self.max_att = 0.10

        # timer rate for publishing motor cmds
        self.DT = 0.01  # 100 Hz

        # ---- State ----
        self.T = 0.0  # base thrust 0..1

        self.roll = 0.0
        self.pitch = 0.0
        self.roll_prev = 0.0
        self.pitch_prev = 0.0
        self.t_prev = None

        self.p = 0.0  # roll rate estimate
        self.q = 0.0  # pitch rate estimate

        # ---- ROS ----
        self.thrust_sub = self.create_subscription(Float64, self.thrust_topic, self.thrust_cb, 10)
        self.pose_sub = self.create_subscription(PoseArray, self.pose_topic, self.pose_cb, 10)
        self.pub = self.create_publisher(Float64MultiArray, self.motor_topic, 10)

        self.timer = self.create_timer(self.DT, self.step)

        self.get_logger().info(
            f"PoseStabilizerMixer:\n"
            f"  pose: {self.pose_topic} (index {self.pose_index})\n"
            f"  thrust: {self.thrust_topic}\n"
            f"  out: {self.motor_topic} as [FR, BL, FL, BR]\n"
        )

    def thrust_cb(self, msg: Float64):
        self.T = clamp(float(msg.data), 0.0, 1.0)

    def pose_cb(self, msg: PoseArray):
        if len(msg.poses) <= self.pose_index:
            return

        pose = msg.poses[self.pose_index]
        q = pose.orientation
        roll, pitch, _ = quat_to_euler(q.x, q.y, q.z, q.w)

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t_prev is None:
            self.t_prev = now
            self.roll_prev = roll
            self.pitch_prev = pitch
            self.roll = roll
            self.pitch = pitch
            self.p = 0.0
            self.q = 0.0
            return

        dt = max(1e-4, now - self.t_prev)

        # unwrap deltas to avoid pi jumps
        droll = angle_diff(roll, self.roll_prev)
        dpitch = angle_diff(pitch, self.pitch_prev)

        # finite-difference rates
        p_est = droll / dt
        q_est = dpitch / dt

        # light low-pass to reduce noise
        alpha = 0.4
        self.p = (1 - alpha) * self.p + alpha * p_est
        self.q = (1 - alpha) * self.q + alpha * q_est

        self.roll = roll
        self.pitch = pitch
        self.roll_prev = roll
        self.pitch_prev = pitch
        self.t_prev = now

    def step(self):
        # errors (want level)
        e_roll = self.roll_sp - self.roll
        e_pitch = self.pitch_sp - self.pitch

        # PD
        R = self.Kp_roll * e_roll - self.Kd_roll * self.p
        P = self.Kp_pitch * e_pitch - self.Kd_pitch * self.q

        R = clamp(R, -self.max_att, self.max_att)
        P = clamp(P, -self.max_att, self.max_att)

        T = self.T

        # ---- Mixer (your verified motor order) ----
        # m0=FR, m1=BL, m2=FL, m3=BR
        FR = T - R - P
        BL = T + R + P
        FL = T + R - P
        BR = T - R + P

        motors = [clamp(x, 0.0, 1.0) for x in (FR, BL, FL, BR)]

        out = Float64MultiArray()
        out.data = motors
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PoseStabilizerMixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()