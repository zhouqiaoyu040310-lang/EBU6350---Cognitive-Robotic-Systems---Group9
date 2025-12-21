# Project Objectives

This project aims to develop an autonomous mobile robot system based on the Yahboom MicroROS-ESP32 platform, capable of autonomous navigation in a maze environment with known topology. The core project objectives include:

## 1. Localization and Mapping Objectives

- **Implement Simultaneous Localization and Mapping (SLAM)**: Utilize MicroROS or ROS2 bridge technology to integrate LIDAR sensors and camera modules, enabling real-time localization and map construction of the robot within the maze environment.

- **Sensor Fusion and Pose Estimation**: Achieve high-precision robot pose estimation by fusing LIDAR point cloud data and camera image frames, with target localization error controlled within 5 cm and orientation error within 3 degrees.

- **Initial Position Identification**: Automatically identify and locate the starting position within the known map, with initial localization time controlled within 5 seconds.

## 2. Path Planning Objectives

- **Optimal Path Planning**: Integrate classical path planning algorithms (such as Dijkstra, A*, RRT, etc.) to achieve optimal or shortest path planning from the starting position to a predefined goal position, with path planning time controlled within 2 seconds.

- **Path Optimization**: Ensure that the ratio of actual path length to theoretical shortest path does not exceed 120%, with path replanning latency controlled within 1 second.

- **Dynamic Obstacle Avoidance**: Real-time obstacle detection during navigation, dynamically adjusting the path to ensure safe passage.

## 3. Navigation and Control Objectives

- **Autonomous Navigation**: The robot should be capable of autonomously navigating along the planned shortest path while avoiding collisions with lines and boundary walls within the maze, with a target success rate ≥ 90%.

- **Real-time Control Algorithm**: Develop a PID-based velocity control algorithm to achieve smooth and stable trajectory tracking, with trajectory deviation controlled within 5 cm RMS.

- **Performance Optimization**: While ensuring stability, achieve an average velocity of at least 0.25 m/s, with actual time-to-goal not exceeding 1.2 times the planned time.

## 4. System Integration and Reliability Objectives

- **Hardware-Software Integration**: Complete robot hardware assembly and sensor calibration, enabling collaborative operation of sensors including LIDAR, camera, IMU, etc., with sensor fusion timestamp error controlled within 50 milliseconds.

- **System Stability**: Ensure continuous system operation time ≥ 20 minutes, ROS2 message packet loss rate ≤ 2%, ensuring reliable system response.

- **Safety Compliance**: Implement emergency stop functionality, control collision rate within 2%, ensuring robot safety during operation.

## 5. Evaluation and Demonstration Objectives

- **Performance Evaluation**: Establish a comprehensive performance evaluation system to record and analyze robot navigation performance metrics, including distance error, path time, success rate, and other objective indicators.

- **Simulation Validation**: Validate the effectiveness of SLAM, path planning, and navigation algorithms in Gazebo/RViz simulation environments, ensuring algorithm reliability before real-world deployment.

- **Practical Demonstration**: Complete at least two full navigation demonstrations from random starting positions to goal positions, showcasing the system's autonomous navigation capabilities and obstacle avoidance effectiveness through video recordings.

## 6. Learning and Reflection Objectives

- **Technical Competence Enhancement**: Master the application of ROS2/MicroROS in embedded robot systems, understand the core principles of sensor fusion, path planning, and autonomous navigation.

- **Engineering Practice Capability**: Through the CDIO design methodology, comprehensively enhance robot system design and implementation capabilities from conceptual design to actual deployment.

- **Critical Thinking**: Transparently record and critically evaluate the use of AI-assisted tools, reflecting on ethical, safety, and bias issues in system design.

By achieving the above objectives, this project will construct a complete autonomous navigation robot system, laying a solid foundation for subsequent more complex robot applications.


