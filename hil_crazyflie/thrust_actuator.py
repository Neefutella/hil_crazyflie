import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from actuator_msgs.msg import Actuators


class MotorCmdsToActuators(Node):
    def __init__(self):
        super().__init__('motor_cmds_to_actuators')

        self.max_motor_speed = 1340.0  # tune (rad/s)
        self.sub = self.create_subscription(
            Float64MultiArray,
            '/motor_cmds',
            self.cb,
            10
        )
        self.pub = self.create_publisher(
            Actuators,
            '/X3/gazebo/command/motor_speed',
            10
        )
        self.get_logger().info("Listening on /motor_cmds -> publishing Actuators.velocity")

    def cb(self, msg: Float64MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn("motor_cmds needs 4 values")
            return

        # clamp each motor command into [0,1]
        m = [max(0.0, min(1.0, float(x))) for x in msg.data[:4]]

        act = Actuators()
        act.velocity = [mi * self.max_motor_speed for mi in m]
        if hasattr(act, "normalized"):
            act.normalized = m
        self.pub.publish(act)



def main(args=None):
    rclpy.init(args=args)
    node = MotorCmdsToActuators()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
