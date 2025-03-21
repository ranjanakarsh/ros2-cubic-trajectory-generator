import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='ar_test',
            executable='points_generator',
            name='points_generator',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='ar_test',
            executable='cubic_traj_planner',
            name='cubic_traj_planner',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='ar_test',
            executable='compute_cubic_coeffs',
            name='compute_cubic_coeffs',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='ar_test',
            executable='plot_cubic_traj',
            name='plot_cubic_traj',
            output='screen'
        ),
    ])
