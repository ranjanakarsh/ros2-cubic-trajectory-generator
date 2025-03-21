import rclpy
from rclpy.node import Node

from ar_interface.msg import CubicTrajParams, CubicTrajCoeffs
from ar_interface.srv import ComputeCubicTraj


class CubicTrajPlanner(Node):
    def __init__(self):
        super().__init__('cubic_traj_planner')

        self.publisher = self.create_publisher(CubicTrajCoeffs, 'cubic_traj_coeffs', 10)
        self.subscription = self.create_subscription(
            CubicTrajParams,
            'cubic_traj_params',
            self.callback_params_received,
            10
        )

        self.client = self.create_client(ComputeCubicTraj, 'compute_cubic_traj')

        self.get_logger().info("cubic_traj_planner node started and waiting for trajectory params")

    def callback_params_received(self, msg):
        if not self.client.service_is_ready():
            self.get_logger().warn("Service not available yet. Waiting...")
            self.client.wait_for_service()

        request = ComputeCubicTraj.Request()
        request.p0 = msg.p0
        request.pf = msg.pf
        request.v0 = msg.v0
        request.vf = msg.vf
        request.t0 = msg.t0
        request.tf = msg.tf

        self.get_logger().info("Calling service with new trajectory parameters...")
        future = self.client.call_async(request)
        future.add_done_callback(lambda future: self.handle_service_response(future, msg))

    def handle_service_response(self, future, msg):
        if future.result() is not None:
            response = future.result()
            coeffs_msg = CubicTrajCoeffs()
            coeffs_msg.a0 = response.a0
            coeffs_msg.a1 = response.a1
            coeffs_msg.a2 = response.a2
            coeffs_msg.a3 = response.a3
            coeffs_msg.t0 = msg.t0
            coeffs_msg.tf = msg.tf

            self.publisher.publish(coeffs_msg)

            self.get_logger().info(
                f"Published coefficients: a0={response.a0:.2f}, a1={response.a1:.2f}, "
                f"a2={response.a2:.2f}, a3={response.a3:.2f}, t0={msg.t0}, tf={msg.tf}"
            )
        else:
            self.get_logger().error("Service call failed!")


def main(args=None):
    rclpy.init(args=args)
    node = CubicTrajPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
