Autonomous Trajectory Tracking Using Path Smoothing and Pure Pursuit (ROS + TurtleBot3)
1. Overview This project implements a complete autonomous navigation pipeline in ROS Noetic
using path smoothing, trajectory generation, and a Pure Pursuit controller executed on TurtleBot3 in
Gazebo simulation.
2. Setup Instructions - Install ROS Noetic and TurtleBot3 simulation packages. - Clone the project
repository into a catkin workspace. - Build using catkin_make. - Run Gazebo simulation and launch
the autonomy stack.
3. Design Choices & Architecture - Cubic Hermite spline interpolation provides smooth global paths.
- Trajectory generation adds dynamic feasibility using curvature-based velocity limits. - Pure Pursuit
is chosen due to its simplicity and suitability for differential drive robots.
4. Extending to a Real Robot - Replace /odom with wheel encoder + IMU fusion. - Add localization,
mapping, safety layers, and hardware motor drivers.
5. AI Tools Used ChatGPT was used for generating boilerplate ROS code, debugging, architectural
explanations, and documentation.
6. Extra Credit: Obstacle Avoidance Obstacle avoidance can be added using local planners such as
DWA or TEB, or a simple LiDAR-based stop mechanism
