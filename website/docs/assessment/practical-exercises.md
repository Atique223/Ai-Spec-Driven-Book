---
sidebar_position: 2
title: Practical Exercises and Projects
---

# Practical Exercises and Projects

This section provides hands-on exercises and projects to reinforce your understanding of Physical AI and Humanoid Robotics concepts. Each exercise builds on the theoretical knowledge with practical implementation.

## Exercise 1: ROS 2 Basics - TurtleBot3 Simulation

### Objective
Learn basic ROS 2 concepts by controlling a simulated TurtleBot3 robot.

### Setup
1. Install ROS 2 (Humble Hawksbill) and Gazebo
2. Install TurtleBot3 packages:
   ```bash
   sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-gazebo
   ```

### Steps
1. Set simulation environment variables:
   ```bash
   export TURTLEBOT3_MODEL=burger
   ```

2. Launch Gazebo simulation:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

3. In a new terminal, send velocity commands:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
   ```

4. Create a simple Python node that makes the robot move in a square pattern:
   ```python
   #!/usr/bin/env python3
   import rclpy
   from rclpy.node import Node
   from geometry_msgs.msg import Twist
   import time

   class SquareMover(Node):
       def __init__(self):
           super().__init__('square_mover')
           self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
           self.timer = self.create_timer(0.1, self.move_square)
           self.state = 0
           self.state_start_time = self.get_clock().now()

       def move_square(self):
           msg = Twist()
           current_time = self.get_clock().now()

           if self.state == 0:  # Move forward
               msg.linear.x = 0.5
               if (current_time - self.state_start_time).nanoseconds > 4e9:
                   self.state = 1
                   self.state_start_time = current_time
           elif self.state == 1:  # Turn
               msg.angular.z = 0.5
               if (current_time - self.state_start_time).nanoseconds > 3.14e9:
                   self.state = 0
                   self.state_start_time = current_time

           self.publisher.publish(msg)

   def main(args=None):
       rclpy.init(args=args)
       square_mover = SquareMover()
       rclpy.spin(square_mover)
       square_mover.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

### Learning Outcomes
- Understanding ROS 2 topics and messages
- Working with geometry_msgs
- Basic robot control concepts

## Exercise 2: Sensor Data Processing

### Objective
Process sensor data from a simulated robot to detect obstacles.

### Setup
Use the same TurtleBot3 simulation from Exercise 1.

### Steps
1. Subscribe to the laser scan topic:
   ```python
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import LaserScan
   from geometry_msgs.msg import Twist

   class ObstacleDetector(Node):
       def __init__(self):
           super().__init__('obstacle_detector')
           self.subscription = self.create_subscription(
               LaserScan,
               '/scan',
               self.scan_callback,
               10)
           self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

       def scan_callback(self, msg):
           # Check for obstacles in front of the robot
           front_scan = msg.ranges[0:30] + msg.ranges[-30:]
           min_distance = min(front_scan)

           cmd_msg = Twist()
           if min_distance < 1.0:  # Obstacle within 1 meter
               cmd_msg.linear.x = 0.0  # Stop
               cmd_msg.angular.z = 0.5  # Turn
           else:
               cmd_msg.linear.x = 0.3  # Move forward
               cmd_msg.angular.z = 0.0

           self.publisher.publish(cmd_msg)

   def main(args=None):
       rclpy.init(args=args)
       detector = ObstacleDetector()
       rclpy.spin(detector)
       detector.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Run the obstacle detector alongside the simulation

### Learning Outcomes
- Processing sensor data in real-time
- Implementing basic navigation behaviors
- Understanding sensor limitations and noise

## Exercise 3: URDF Robot Model Creation

### Objective
Create a custom robot model using URDF and visualize it in Gazebo.

### Steps
1. Create a URDF file for a simple differential drive robot:
   ```xml
   <?xml version="1.0"?>
   <robot name="custom_robot">
     <!-- Base Link -->
     <link name="base_link">
       <visual>
         <geometry>
           <cylinder radius="0.2" length="0.1"/>
         </geometry>
         <material name="blue">
           <color rgba="0 0 1 0.8"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <cylinder radius="0.2" length="0.1"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="5.0"/>
         <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.2"/>
       </inertial>
     </link>

     <!-- Left Wheel -->
     <link name="left_wheel">
       <visual>
         <geometry>
           <cylinder radius="0.1" length="0.05"/>
         </geometry>
         <material name="black">
           <color rgba="0 0 0 0.8"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <cylinder radius="0.1" length="0.05"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.5"/>
         <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
       </inertial>
     </link>

     <!-- Right Wheel -->
     <link name="right_wheel">
       <visual>
         <geometry>
           <cylinder radius="0.1" length="0.05"/>
         </geometry>
         <material name="black">
           <color rgba="0 0 0 0.8"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <cylinder radius="0.1" length="0.05"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.5"/>
         <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
       </inertial>
     </link>

     <!-- Joints -->
     <joint name="left_wheel_joint" type="continuous">
       <parent link="base_link"/>
       <child link="left_wheel"/>
       <origin xyz="0 0.15 -0.05" rpy="1.57079 0 0"/>
       <axis xyz="0 0 1"/>
     </joint>

     <joint name="right_wheel_joint" type="continuous">
       <parent link="base_link"/>
       <child link="right_wheel"/>
       <origin xyz="0 -0.15 -0.05" rpy="1.57079 0 0"/>
       <axis xyz="0 0 1"/>
     </joint>
   </robot>
   ```

2. Create a launch file to spawn the robot in Gazebo
3. Visualize the robot in RViz and Gazebo

### Learning Outcomes
- Understanding URDF structure and components
- Creating 3D robot models
- Working with joints and kinematics

## Exercise 4: AI Integration with Robot Control

### Objective
Integrate a simple AI algorithm (wall following) with robot control.

### Steps
1. Create a wall-following node that uses laser scan data:
   ```python
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import LaserScan
   from geometry_msgs.msg import Twist
   import math

   class WallFollower(Node):
       def __init__(self):
           super().__init__('wall_follower')
           self.subscription = self.create_subscription(
               LaserScan,
               '/scan',
               self.scan_callback,
               10)
           self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
           self.target_distance = 0.5  # Desired distance from wall

       def scan_callback(self, msg):
           # Get relevant ranges (right side of robot)
           right_ranges = msg.ranges[270:330]  # 60-degree arc on right
           front_ranges = msg.ranges[330:30] + msg.ranges[0:30]  # Front ranges

           # Calculate average distances
           right_avg = sum(r for r in right_ranges if not math.isnan(r) and r > 0) / len([r for r in right_ranges if not math.isnan(r) and r > 0])
           front_avg = min([r for r in front_ranges if not math.isnan(r) and r > 0])

           cmd_msg = Twist()

           # Wall following logic
           if front_avg < 0.6:  # Too close to front obstacle
               cmd_msg.angular.z = 0.5  # Turn right
           elif right_avg < self.target_distance * 0.8:  # Too close to wall
               cmd_msg.linear.x = 0.2
               cmd_msg.angular.z = 0.3  # Turn left to move away
           elif right_avg > self.target_distance * 1.2:  # Too far from wall
               cmd_msg.linear.x = 0.2
               cmd_msg.angular.z = -0.3  # Turn right to move closer
           else:  # Good distance from wall
               cmd_msg.linear.x = 0.3
               cmd_msg.angular.z = 0.0

           self.publisher.publish(cmd_msg)

   def main(args=None):
       rclpy.init(args=args)
       follower = WallFollower()
       rclpy.spin(follower)
       follower.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

### Learning Outcomes
- Integrating AI algorithms with robot control
- Processing sensor data for decision making
- Implementing reactive behaviors

## Exercise 5: Simulation-Reality Transfer Concepts

### Objective
Understand the challenges of transferring from simulation to reality.

### Steps
1. Create a simple navigation task in simulation
2. Add noise to sensor data to simulate real-world imperfections
3. Implement a robust control algorithm that works with noisy data
4. Compare performance with and without noise

### Learning Outcomes
- Understanding the sim-to-real gap
- Developing robust algorithms
- Handling sensor uncertainty

## Project 1: Autonomous Navigation

### Objective
Create a complete autonomous navigation system for a simulated robot.

### Requirements
1. Map an unknown environment using SLAM
2. Plan paths to user-specified goals
3. Avoid obstacles in real-time
4. Execute navigation safely

### Implementation Steps
1. Set up a robot with LIDAR in Gazebo
2. Implement SLAM (use ROS 2 navigation stack)
3. Create a path planning node
4. Implement obstacle avoidance
5. Test in various environments

### Evaluation Criteria
- Success rate in reaching goals
- Time efficiency
- Safety (no collisions)
- Robustness to environmental changes

## Project 2: Humanoid Robot Balance Control

### Objective
Implement a basic balance control system for a simulated humanoid robot.

### Requirements
1. Maintain balance when standing
2. Recover from small disturbances
3. Provide visualization of center of mass

### Implementation Steps
1. Create or import a humanoid robot model
2. Implement center of mass estimation
3. Design a balance controller (e.g., PID or LQR)
4. Test with external disturbances
5. Visualize balance metrics

### Learning Outcomes
- Understanding humanoid robot kinematics
- Implementing balance control algorithms
- Working with complex multi-DOF systems

## Project 3: Human-Robot Interaction Interface

### Objective
Create an interactive interface for commanding a robot using Unity.

### Requirements
1. Unity visualization of the robot
2. User input for commanding robot movement
3. Real-time feedback of robot state
4. Safety features in the interface

### Implementation Steps
1. Set up Unity with ROS# connection
2. Create robot visualization in Unity
3. Implement user input handling
4. Connect to ROS for robot control
5. Add safety validation to commands

### Learning Outcomes
- Unity-ROS integration
- Human-robot interaction design
- Visualization techniques

## Assessment Rubric

### Technical Implementation (40%)
- Correctness of code and algorithms
- Proper use of ROS 2 concepts
- Efficiency and optimization

### Problem-Solving (30%)
- Creative solutions to challenges
- Understanding of underlying concepts
- Troubleshooting and debugging skills

### Documentation (20%)
- Clear code comments
- Proper README files
- Explanation of design decisions

### Functionality (10%)
- System works as intended
- Meets specified requirements
- Robust operation

## Additional Resources

### Online Tutorials
- ROS 2 documentation and tutorials
- Gazebo simulation tutorials
- Unity robotics packages documentation
- GitHub repositories with robotics examples

### Tools and Libraries
- RViz for visualization
- rqt tools for debugging
- Gazebo plugins for sensors
- Unity Robotics Package

### Communities
- ROS Discourse
- Gazebo community
- Unity robotics forums
- Robotics Stack Exchange

## Extension Activities

### Advanced Challenges
1. Implement machine learning for robot control
2. Create multi-robot coordination systems
3. Develop advanced manipulation skills
4. Integrate multiple AI modalities (vision, language, planning)

### Research Directions
1. Study recent papers in Physical AI
2. Explore state-of-the-art humanoid robots
3. Investigate new simulation techniques
4. Research human-robot interaction methods

These exercises and projects provide a comprehensive pathway from basic concepts to advanced implementations in Physical AI and Humanoid Robotics. Complete them in order to build a solid foundation, or select specific exercises based on your interests and learning objectives.