Localization-and-Navigation-Using-Extended-Kalman-Filter

# **Project Objective:**

Developed a robust localization and navigation framework for Micro Aerial Vehicles (MAVs) using an Extended Kalman Filter (EKF) to estimate key state variables such as position, velocity, and orientation. The project focuses on sensor fusion of IMU data and Vicon measurements to enhance state estimation accuracy, even in noisy and dynamic environments.

# **Project Overview:**

Localization and navigation are crucial for autonomous MAVs, and this project implements a comprehensive EKF-based sensor fusion system to address these challenges. By combining high-frequency IMU data with reliable Vicon measurements, the system accurately tracks the MAV’s position and velocity while compensating for sensor noise and packet loss. The project is divided into two major parts:

- **Part 1:** Estimating position and orientation using Vicon pose data.
- **Part 2:** Estimating velocity using Vicon velocity data.

# **Key Contributions:**

- **Robust EKF Implementation:** Built a dynamic EKF system that uses body-frame acceleration and angular velocity to predict the MAV’s state, followed by updates from position and velocity measurements.
- **Multi-Sensor Fusion:** Successfully fused noisy, asynchronous IMU and Vicon data, ensuring robust and consistent state estimation.
- **Efficient State Updates:** Integrated separate measurement models for position, orientation, and velocity, optimizing accuracy through Kalman gain-based updates.
- **Visualization and Error Analysis:** Visualized estimation accuracy by comparing EKF outputs against Vicon ground truth, analyzing errors in position, velocity, and orientation.

# **Methodology:**

**1. Prediction Step:**

- Propagated state variables (position, velocity, orientation) using IMU-derived acceleration and angular velocity.
- Predicted the next state using a process model and updated the covariance matrix to account for uncertainty.

**2. Update Step:**

- Part 1: Incorporated Vicon position and orientation measurements to correct the state estimates.
- Part 2: Used Vicon velocity data to refine velocity estimates while updating the covariance matrix.
- Calculated Kalman gain to balance trust between predictions and noisy measurements.

**3. Visualization:**

- Plotted positional, velocity, and orientation estimates against Vicon ground truth data to evaluate performance across different motion scenarios.

#**Challenges and Solutions:**

- **Handling Sensor Noise:** Integrated noise models and covariance adjustments to mitigate the effects of sensor noise and measurement inconsistencies.
- **Dealing with Packet Loss:** Designed the EKF to handle asynchronous updates by fusing data streams with varying latencies.
- **Maintaining Real-Time Performance:** Optimized the MATLAB codebase for efficient computation of prediction and update steps, enabling near real-time performance.

# **Key Outcomes:**

- Accurate tracking of MAV position and velocity across different datasets and motion profiles.
- Error margins for positional estimates were minimized through optimized Kalman gain and robust sensor fusion.
- Plots showcased minimal deviation between EKF estimates and ground truth, ensuring reliable localization.

# **Technologies and Tools:**

Matlab

- **State Estimation:** Extended Kalman Filter (EKF)
- **Programming and Visualization:** MATLAB
- **Sensors:** IMU, Vicon motion capture system
- **Data Structures:** .mat files containing synchronized IMU and Vicon data for real-world evaluation

# **Future Improvements:**

- Incorporate visual odometry data to further improve localization in dynamic environments.
- Extend the system for SLAM (Simultaneous Localization and Mapping) with real-time obstacle detection.
- Test additional datasets with varying noise and motion dynamics to generalize the system across diverse scenarios.

This project delivers a reliable framework for autonomous navigation and localization, making it ideal for applications in UAVs, robotics research, and aerospace systems.
