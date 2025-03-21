import rclpy
import random
from rclpy.node import Node
from ar_interface.msg import CubicTrajParams

class PointsGenerator(Node):
    def __init__(self):
        super().__init__('points_generator')
        self.publisher = self.create_publisher(CubicTrajParams, 'cubic_traj_params', 10)
        self.timer = self.create_timer(10.0, self.publish_random_trajectory)
        self.get_logger().info("points_generator node started")

    def publish_random_trajectory(self):
        msg = CubicTrajParams()
        msg.p0 = random.uniform(-10.0, 10.0)
        msg.pf = random.uniform(-10.0, 10.0)
        msg.v0 = random.uniform(-10.0, 10.0)
        msg.vf = random.uniform(-10.0, 10.0)
        msg.t0 = 0.0
        msg.tf = random.uniform(4.0, 8.0)

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published CubicTrajParams: p0={msg.p0:.2f}, pf={msg.pf:.2f}, v0={msg.v0:.2f}, vf={msg.vf:.2f}, t0={msg.t0}, tf={msg.tf:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PointsGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
