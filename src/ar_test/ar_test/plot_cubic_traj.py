import rclpy
import numpy as np
from rclpy.node import Node
from ar_interface.msg import CubicTrajCoeffs
from std_msgs.msg import Float64MultiArray


class PlotCubicTraj(Node):
    def __init__(self):
        super().__init__('plot_cubic_traj')
        
        # Publishers for position, velocity, acceleration
        self.position_publisher = self.create_publisher(Float64MultiArray, 'cubic_traj_position', 10)
        self.velocity_publisher = self.create_publisher(Float64MultiArray, 'cubic_traj_velocity', 10)
        self.acceleration_publisher = self.create_publisher(Float64MultiArray, 'cubic_traj_acceleration', 10)
        
        # Subscriber for computed cubic coefficients
        self.subscription = self.create_subscription(
                                                     CubicTrajCoeffs,
                                                     'cubic_traj_coeffs',
                                                     self.trajectory_callback,
                                                     10
                                                     )
                                                     
        self.get_logger().info("plot_cubic_traj node started, waiting for trajectory coefficients...")
    
    def trajectory_callback(self, msg):
        self.get_logger().info(f"Received Coefficients: a0={msg.a0:.2f}, a1={msg.a1:.2f}, a2={msg.a2:.2f}, a3={msg.a3:.2f}")
        
        # Generate time steps
        t_values = np.linspace(msg.t0, msg.tf, num=200)
        
        # Compute position, velocity, acceleration at each time step
        positions = msg.a0 + msg.a1 * t_values + msg.a2 * t_values**2 + msg.a3 * t_values**3
        velocities = msg.a1 + 2 * msg.a2 * t_values + 3 * msg.a3 * t_values**2
        accelerations = 2 * msg.a2 + 6 * msg.a3 * t_values
        
        # Convert to ROS messages
        position_msg = Float64MultiArray(data=positions.tolist())
        velocity_msg = Float64MultiArray(data=velocities.tolist())
        acceleration_msg = Float64MultiArray(data=accelerations.tolist())
        
        # Publish data
        for i, position in enumerate(position_msg.data):
            msg = Float64MultiArray()
            msg.data = [position]  # Publish one value at a time
            self.position_publisher.publish(msg)
            self.get_logger().info(f"Published position[{i}]: {position:.3f}")
            
        for i, position in enumerate(velocity_msg.data):
            msg = Float64MultiArray()
            msg.data = [position]  # Publish one value at a time
            self.velocity_publisher.publish(msg)
            self.get_logger().info(f"Velocity position[{i}]: {position:.3f}")

        for i, position in enumerate(acceleration_msg.data):
            msg = Float64MultiArray()
            msg.data = [position]  # Publish one value at a time
            self.acceleration_publisher.publish(msg)
            self.get_logger().info(f"Acceleration position[{i}]: {position:.3f}")
        
        self.get_logger().info("Published position, velocity, and acceleration data")


def main(args=None):
    rclpy.init(args=args)
    node = PlotCubicTraj()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
