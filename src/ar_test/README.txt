Name: Ranjan Akarsh
Student ID: 231229968

Installation Instructions:
1. Clone the repository:
   git clone [your_repo_url] ~/ros2_ws/src/ar_test

2. Build the workspace:
   cd ~/ros2_ws
   colcon build --packages-select ar_test --symlink-install

3. Source the environment:
   source install/setup.zsh

4. Run the full pipeline:
   ros2 launch ar_test cubic_traj_gen.launch.py

Usage:
- The nodes will generate and compute cubic trajectories automatically.
- Open rqt_plot and add the following topics:
  /cubic_traj_position/data[0]
  /cubic_traj_velocity/data[0]
  /cubic_traj_acceleration/data[0]