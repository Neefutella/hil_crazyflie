import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        # Publish per-motor normalized commands in [0,1]
        self.pub = self.create_publisher(Float64MultiArray, '/motor_cmds', 10)

        # Commands
        self.thrust = 0.0   # base thrust [0..1]
        self.roll = 0.0     # left/right tilt command [-..+]
        self.pitch = 0.0    # forward/back tilt command [-..+]

        # Step sizes (tune)
        self.thrust_step = 0.01
        self.att_step = 0.01

        # Limits (keep attitude small to avoid flips)
        self.max_thrust = 1.0
        self.max_att = 0.05   # roll/pitch max magnitude

        self.get_logger().info(
            "Keyboard teleop:\n"
            "  Arrow Up/Down : thrust up/down\n"
            "  W/S           : forward/back (pitch)\n"
            "  A/D           : left/right (roll)\n"
            "  Space         : cut motors\n"
            "  Q             : quit\n"
            "\n"
            "Tip: tap inputs (don’t hold) until you feel the dynamics."
        )

        # 20 Hz control publish loop
        self.timer = self.create_timer(0.05, self.step)

        # terminal settings
        self.fd = sys.stdin.fileno()
        self.old_term_settings = termios.tcgetattr(self.fd)

    def destroy_node(self):
        # restore terminal on exit
        try:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term_settings)
        except Exception:
            pass
        super().destroy_node()

    def read_key(self):
        """Read a key (supports arrow keys). Returns:
           'UP','DOWN','LEFT','RIGHT' or single-char like 'w','a',' '.
        """
        tty.setraw(self.fd)
        ch1 = sys.stdin.read(1)
        if ch1 != '\x1b':
            return ch1  # normal key

        # possible escape sequence for arrow keys
        ch2 = sys.stdin.read(1)
        if ch2 != '[':
            return ch1

        ch3 = sys.stdin.read(1)
        if ch3 == 'A':
            return 'UP'
        if ch3 == 'B':
            return 'DOWN'
        if ch3 == 'C':
            return 'RIGHT'
        if ch3 == 'D':
            return 'LEFT'
        return ch1

    def step(self):
        # restore cooked mode briefly between reads
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term_settings)

        # read one key per tick (simple + predictable)
        key = self.read_key()

        # restore settings immediately after read
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term_settings)

        # ---- Key mappings ----
        if key == 'UP':
            self.thrust += self.thrust_step
        elif key == 'DOWN':
            self.thrust -= self.thrust_step

        elif key in ('a', 'A'):
            self.roll -= self.att_step
        elif key in ('d', 'D'):
            self.roll += self.att_step

        elif key in ('w', 'W'):
            self.pitch += self.att_step
        elif key in ('s', 'S'):
            self.pitch -= self.att_step

        elif key == ' ':
            self.thrust = 0.0
            self.roll = 0.0
            self.pitch = 0.0

        elif key in ('q', 'Q'):
            self.get_logger().info("Exiting teleop")
            rclpy.shutdown()
            return

        # clamp commands
        self.thrust = clamp(self.thrust, 0.0, self.max_thrust)
        self.roll = clamp(self.roll, -self.max_att, self.max_att)
        self.pitch = clamp(self.pitch, -self.max_att, self.max_att)
        # auto-center for roll/pitch
        decay = 0.10 # passive return rate
        self.roll *= decay
        self.pitch *= decay

        # ---- Motor mixing (X-config, roll + pitch only) ----
        # Verified Gazebo X3 motor index mapping:
        #   m0 = Front Right (FR)
        #   m1 = Back  Left  (BL)
        #   m2 = Front Left  (FL)
        #   m3 = Back  Right (BR)
        #
        # Convention we enforce:
        #   pitch + : move FORWARD  -> rear motors up, front motors down
        #   roll  + : move RIGHT    -> left motors up, right motors down

        T = self.thrust
        R = self.roll    # + => right
        P = self.pitch   # + => forward

        FR = T - R - P
        BL = T + R + P
        FL = T + R - P
        BR = T - R + P

        motors = [clamp(m, 0.0, 1.0) for m in (FR, BL, FL, BR)]
        
        msg = Float64MultiArray()
        msg.data = motors
        self.pub.publish(msg)

        self.get_logger().info(
            f"T={self.thrust:.2f} R={self.roll:.2f} P={self.pitch:.2f}  "
            f"m={[f'{m:.2f}' for m in motors]}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
