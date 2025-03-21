# ROS 2 Cubic Trajectory Generator

🚀 A ROS2 pipeline for generating, computing, and visualizing cubic motion trajectories using custom messages, services, and live plotting.

---

## 📚 Overview

This project was developed for the **EMS628U728** coursework and implements a modular ROS 2 system with:

- **Custom service and message interfaces**
- **Trajectory parameter generation**
- **Coefficient computation**
- **Trajectory reconstruction and visualization**

The full system is launched using a single launch file and visualized with `rqt_plot`.

---

## 📦 Package: `ar_test`

### 📁 File Structure

```bash
ar_test/
├── ar_test/
│   ├── points_generator.py
│   ├── compute_cubic_coeffs.py
│   ├── cubic_traj_planner.py
│   ├── plot_cubic_traj.py
├── launch/
│   └── cubic_traj_gen.launch.py
├── msg/
│   ├── CubicTrajParams.msg
│   └── CubicTrajCoeffs.msg
├── srv/
│   └── ComputeCubicTraj.srv
├── package.xml
├── setup.py
└── README.txt
```

## Node Descriptions

| Node Name | Description |
| :---:   | :---: |
| points_generator | Publishes random initial/final positions & velocities every 10 seconds. |
| compute_cubic_coeffs | Provides a service to compute cubic trajectory coefficients. |
| cubic_traj_planner | Calls the service and publishes the computed coefficients. |
| plot_cubic_traj | Subscribes to coefficients and publishes position, velocity, and acceleration for plotting. | 

## Building the package
```bash
cd ~/ros2_ws # the directory you put 'src' in 
colcon build --packages-select ar_test --symlink-install
source install/setup.zsh
```

## Launching the system
```bash
ros2 launch ar_test cubic_traj_gen.launch.py
```
This command runs all four nodes.

### Visualizing the plot
```bash
rqt_plot # or just 'rqt' then select Plugins -> Visualization -> Plot
```

Then enter these topics:
	•	_/cubic_traj_position/data[0]_
	•	_/cubic_traj_velocity/data[0]_
	•	_/cubic_traj_acceleration/data[0]_
