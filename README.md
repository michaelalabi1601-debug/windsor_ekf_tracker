# ROS 2 2D Extended Kalman Filter (EKF) Localization Pipeline

A robust, custom-built Extended Kalman Filter (EKF) localization and state estimation pipeline implemented from scratch in Python for **ROS 2 Humble**. This project models a mobile robot tracking a circular trajectory under noisy sensor measurements, featuring real-time state prediction, analytical Jacobian linearization, and persistent trajectory visualization.

---

## 🚀 Key Features

* **Custom EKF Engine:** Built from the ground up using `NumPy` without relying on black-box filter packages, demonstrating a deep, foundational understanding of recursive state estimation.
* **Nonlinear Unicycle Kinematics:** Implements a 2D unicycle motion model incorporating linear velocity ($v$), angular velocity ($\omega$), and heading angle ($\theta$).
* **Analytical Jacobian Linearization:** Derives and implements state transition Jacobians ($F_k$) and measurement matrices ($H_k$) for optimal covariance propagation and gain tuning.
* **Persistent Path Visualization:** Utilizes `nav_msgs/Path` to render a glowing, continuous trajectory trail in **RViz2** alongside orientation-corrected `geometry_msgs/PoseStamped` vectors.
* **Modular ROS 2 Architecture:** Clean separation of concerns between trajectory/sensor simulation nodes and state estimation nodes.

---

## 📐 Mathematical Formulation

### 1. State Vector
The system state $x_k$ at time step $k$ is defined as:
$$x_k = \begin{bmatrix} x & y & \theta \end{bmatrix}^T$$
where $(x, y)$ represents the 2D Cartesian position and $\theta$ represents the heading orientation.

### 2. Prediction Step (Motion Model)
Given linear velocity $v$ and angular velocity $\omega$:
$$x_{k|k-1} = x_{k-1} + \begin{bmatrix} v \cos(\theta_{k-1}) \Delta t \\ v \sin(\theta_{k-1}) \Delta t \\ \omega \Delta t \end{bmatrix}$$

The process model Jacobian matrix $F_k = \frac{\partial f}{\partial x}$ is computed analytically as:
$$F_k = \begin{bmatrix} 
1.0 & 0.0 & -v \sin(\theta) \Delta t \\ 
0.0 & 1.0 & v \cos(\theta) \Delta t \\ 
0.0 & 0.0 & 1.0 
\end{bmatrix}$$

Covariance prediction:
$$P_{k|k-1} = F_k P_{k-1} F_k^T + Q$$

### 3. Update Step (Measurement Model)
Using a GPS sensor measuring 2D position $z = \begin{bmatrix} z_x & z_y \end{bmatrix}^T$:
$$H = \begin{bmatrix} 
1.0 & 0.0 & 0.0 \\ 
0.0 & 1.0 & 0.0 
\end{bmatrix}$$

* **Innovation Covariance:** $S = H P_{k|k-1} H^T + R$
* **Kalman Gain:** $K = P_{k|k-1} H^T S^{-1}$
* **State Update:** $x_k = x_{k|k-1} + K (z - H x_{k|k-1})$
* **Covariance Update:** $P_k = (I - K H) P_{k|k-1}$

---

## 🛠️ Tech Stack & Dependencies

* **OS:** Ubuntu 22.04 (via WSL 2)
* **Framework:** ROS 2 Humble Hawksbill
* **Language:** Python 3.10+
* **Core Libraries:** `rclpy`, `numpy`, `geometry_msgs`, `nav_msgs`, `rviz2`

---

## 📦 Installation & Quickstart

### 1. Clone into your ROS 2 Workspace
```bash
cd ~/ros2_ws/src
git clone [https://github.com/michaelalabi1601-debug/windsor_ekf_tracker.git](https://github.com/michaelalabi1601-debug/windsor_ekf_tracker.git)
```
### 2. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select windsor_ekf_tracker --symlink-install
source install/setup.bash
```
### 3. Run the Simulation Nodes

Open three seperate terminal windows and source your workspace (```source install/setup.bash```) in each;

#### Terminal 1 (Trajectory & GPS Simulator):
```bash
ros2 run windsor_ekf_tracker trajectory_sim
```
#### Terminal 2 (EKF State Estimation Node):
```bash
ros2 run windsor_ekf_tracker ekf_node
```
#### Terminal 3 (Static Transform Publisher for RViz):
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link
```

### 4. Visualize in RViz2
launch RViz2 in a separate terminal:
 ```bash
 rviz2
 ```
  Set the Fixed Frame to ```map```.
  Add displays for **Path** (topic: ```/estimated_path```) and **Pose**(topic: ```/estimated_pose```).

## 💡 Future Improvements
**IMU Sensor Fusion:** Integration of IMU angular velocity measurements into the prediction step for 6-DOF tracking.

**Outlier Rejection:** Implementation of Mahalanobis distance gating for robust rejection of GPS multipath noise.

**Control Loop Closure:** Coupling the estimated state output with a Pure Pursuit or MPC trajectory tracking controller.

*Developed as part of advanced robotics and control systems portfolio projects.* 

