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

| Attempt | #1    | #2    |
| :---:   | :---: | :---: |
| Seconds | 301   | 283   |
