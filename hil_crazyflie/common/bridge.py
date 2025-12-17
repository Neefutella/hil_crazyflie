import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Pose_V
from std_msgs.msg import Float64

class PoseToZ(Node):
    def __init__(self):
        super().__init__('pose_to_z')
        self.sub = self.create_subscription(
            Pose_V,
            '/world/quadcopter/pose/info',
            self.cb,
            10
        )
        self.pub = self.create_publisher(Float64, '/z_meas', 10)
        self.model_name = 'X3'  # or whatever your quad's name is inside that message
        self.get_logger().info("PoseToZ started")

    def cb(self, msg: Pose_V):
        # Pose_V contains names + poses; pick your quad's pose
        for name, pose in zip(msg.name, msg.pose):
            if name == self.model_name:
                z_msg = Float64()
                z_msg.data = pose.position.z
                self.pub.publish(z_msg)
                break

def main(args=None):
    rclpy.init(args=args)
    node = PoseToZ()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()