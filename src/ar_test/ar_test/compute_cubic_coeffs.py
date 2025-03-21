import rclpy
from rclpy.node import Node
from ar_interface.srv import ComputeCubicTraj

class ComputeCubicCoeffs(Node):
    def __init__(self):
        super().__init__('compute_cubic_coeffs')
        self.srv = self.create_service(ComputeCubicTraj, 'compute_cubic_traj', self.compute_coeffs_callback)
        self.get_logger().info("compute_cubic_coeffs service is ready")

    def compute_coeffs_callback(self, request, response):
        p0 = request.p0
        pf = request.pf
        v0 = request.v0
        vf = request.vf
        t0 = request.t0
        tf = request.tf
        T = tf - t0

        response.a0 = p0
        response.a1 = v0
        response.a2 = (3 * (pf - p0) / (T**2)) - (2 * v0 + vf) / T
        response.a3 = (2 * (p0 - pf) / (T**3)) + (v0 + vf) / (T**2)

        self.get_logger().info(
            f"📥 Received: p0={p0}, pf={pf}, v0={v0}, vf={vf}, t0={t0}, tf={tf}\n"
            f"📤 Responded: a0={response.a0:.2f}, a1={response.a1:.2f}, a2={response.a2:.2f}, a3={response.a3:.2f}"
        )

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ComputeCubicCoeffs()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
