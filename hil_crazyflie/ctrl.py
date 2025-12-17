import time
import csv
import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # --- Control parameters ---
        self.DT = 0.01           # 100 Hz
        self.target_z = 1.0      # meters
        self.Kp = 2.00           # position gain
        self.Kd = 0.50           # velocity damping gain

        # normalized thrust command (0-1)
        self.HOVER_CMD = 0.5     # hover
        self.MAX_CMD = 1.0       # max throttle
        self.MIN_CMD = 0.0       # no throttle
        self.MAX_CMD_STEP = 0.5  # max change per step (rate limit)

        # --- State ---
        self.z = 0.0
        self.prev_z = 0.0
        self.first = True
        self.cmd = self.HOVER_CMD  # current command

        # --- ROS2 interfaces ---
        # Input: altitude measurement from plant
        self.z_sub = self.create_subscription(Float64, '/z_meas', self.z_callback, 10)

        # Output: normalized thrust to plant
        self.cmd_pub = self.create_publisher(Float64, '/thrust_cmd', 10)

        # 100 Hz control loop
        self.timer = self.create_timer(self.DT, self.control_step)

        # --- Logging & plotting ---
        self.log = []
        self.start_time = time.time()

        self.time_data = []
        self.z_data = []

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], lw=2)

        self.ax.set_xlabel("time (s)")
        self.ax.set_ylabel("altitude z (m)")
        self.ax.set_title("drone altitude observation")
        self.ax.grid(True)

        plt.show(block=False)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        self.get_logger().info("[CTRL] Controller started")

    # ----------------- Callbacks -----------------

    def z_callback(self, msg: Float64):
        """Receive altitude measurement from plant."""
        self.z = msg.data

    def control_step(self):
        """Run one PD control update at 100 Hz."""
        # time since start (needed for ramp)
        t = time.time() - self.start_time

        # smooth reference: ramp target from 0 -> self.target_z over 1 second
        rise_time = 1.0  # seconds
        if t < rise_time:
            target_z_eff = (t / rise_time) * self.target_z
        else:
            target_z_eff = self.target_z

        # estimate vertical velocity
        if self.first:
            vel_z = 0.0
            self.first = False
        else:
            vel_z = (self.z - self.prev_z) / self.DT

        # use the *effective* target (ramped)
        error = target_z_eff - self.z

        # PD control on altitude
        u = self.Kp * error - self.Kd * vel_z

        # normalized thrust command before limits
        cmd_desired = self.HOVER_CMD + u

        # saturate
        cmd_desired = float(np.clip(cmd_desired, self.MIN_CMD, self.MAX_CMD))

        # rate limit (don't change faster than MAX_CMD_STEP per cycle)
        delta = np.clip(cmd_desired - self.cmd,
                        -self.MAX_CMD_STEP,
                        self.MAX_CMD_STEP)
        self.cmd += delta

        # publish command
        cmd_msg = Float64()
        cmd_msg.data = float(self.cmd)
        self.cmd_pub.publish(cmd_msg)

        # log
        self.log.append((t, self.z))
        self.prev_z = self.z
        
        #store data for plotting
        self.time_data.append(t)
        self.z_data.append(self.z)

        self.get_logger().info(
            f"[CTRL] t={t:5.2f}s  z={self.z:6.3f}  vel_z={vel_z:6.3f}  cmd={self.cmd:4.2f}"
        )

        # update plot every 5 samples
        if len(self.time_data) % 5 == 0:
            self.line.set_data(self.time_data, self.z_data)
            self.ax.relim()
            self.ax.autoscale_view()
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

    # ----------------- Shutdown hook -----------------

    def destroy_node(self):
        """Save log on shutdown, then destroy node."""
        self.get_logger().info("\n[CTRL] Saving log...")
        try:
            with open("z_log.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["time", "z"])
                writer.writerows(self.log)
            self.get_logger().info("[CTRL] Log saved to z_log.csv")
        except Exception as e:
            self.get_logger().error(f"Failed to save log: {e}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
