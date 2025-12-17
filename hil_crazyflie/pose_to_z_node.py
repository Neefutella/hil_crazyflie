import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64

class PoseToZ(Node):
    def __init__(self):
        super().__init__('pose_to_z')
        self.sub = self.create_subscription(
            PoseArray,
            '/world/quadcopter/pose/info',
            self.cb,
            10
        )
        self.pub = self.create_publisher(Float64, '/z_meas', 10)
        self.index = 1   # which pose in the array to use
        self.get_logger().info("PoseToZ started (using poses[1])")

    def cb(self, msg: PoseArray):
        if len(msg.poses) <= self.index:
            return
        pose = msg.poses[self.index]
        out = Float64()
        out.data = pose.position.z
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToZ()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
