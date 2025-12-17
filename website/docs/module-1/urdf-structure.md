---
sidebar_position: 5
title: Understanding URDF for Humanoid Robot Structure
---

# Understanding URDF for Humanoid Robot Structure

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. For humanoid robots, URDF provides a standardized way to define the robot's physical structure, including links, joints, and their relationships.

## What is URDF?

URDF is an XML format that describes a robot in terms of:

- **Links**: Rigid parts of the robot (e.g., torso, arms, legs)
- **Joints**: Connections between links (e.g., hinges, prismatic joints)
- **Visual**: How the robot appears in simulation
- **Collision**: How the robot interacts with the physical world
- **Inertial**: Physical properties for simulation

## Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.25 0" rpy="0 0 0"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## URDF Components for Humanoid Robots

### Links
Links represent rigid bodies in the robot. For a humanoid robot, typical links include:
- Torso/base
- Head
- Upper arms
- Lower arms
- Hands
- Upper legs
- Lower legs
- Feet

### Joints
Joints define how links connect and move relative to each other. Common joint types:
- **revolute**: Rotational joint with limited range
- **continuous**: Rotational joint without limits
- **prismatic**: Linear sliding joint
- **fixed**: No movement between links
- **floating**: 6-DOF movement (rarely used)

### Visual and Collision Properties
```xml
<link name="upper_arm">
  <visual>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://my_robot/meshes/upper_arm.dae"/>
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1.0"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="2.0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
  </inertial>
</link>
```

## Humanoid Robot Skeleton

A typical humanoid robot URDF follows a tree structure:

```
base_link (torso)
├── head
├── upper_left_arm
│   └── lower_left_arm
│       └── left_hand
├── upper_right_arm
│   └── lower_right_arm
│       └── right_hand
├── upper_left_leg
│   └── lower_left_leg
│       └── left_foot
└── upper_right_leg
    └── lower_right_leg
        └── right_foot
```

## URDF Best Practices

1. **Use consistent naming**: Follow conventions like `left_upper_arm` vs `upper_arm_left`
2. **Start with base_link**: Always have a clear root link
3. **Define proper origins**: Joint origins should be at the physical joint location
4. **Include inertial properties**: Important for physics simulation
5. **Use mesh files for complex shapes**: For visual appearance
6. **Simple collision geometry**: Use simple shapes for collision detection

## Xacro for Complex URDFs

For complex humanoid robots, Xacro (XML Macros) helps manage URDF complexity:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">

  <xacro:property name="M_PI" value="3.1415926535897931" />

  <xacro:macro name="simple_arm" params="side">
    <link name="${side}_upper_arm">
      <visual>
        <geometry>
          <cylinder length="0.3" radius="0.05"/>
        </geometry>
      </visual>
    </link>

    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="torso"/>
      <child link="${side}_upper_arm"/>
      <origin xyz="0.2 0 0" rpy="0 0 0"/>
      <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="1"/>
    </joint>
  </xacro:macro>

  <xacro:simple_arm side="left"/>
  <xacro:simple_arm side="right"/>
</robot>
```

## URDF in Simulation

URDF is used by simulation environments like Gazebo to:
- Visualize the robot
- Calculate physics interactions
- Generate collision detection
- Provide forward kinematics

## Knowledge Check

1. What's the difference between visual and collision properties in URDF?
2. Why are inertial properties important in URDF?
3. How does the tree structure of URDF links relate to the robot's kinematic chain?

## Practical Exercise

Create a simplified URDF for a basic humanoid robot with at least 10 links (torso, head, 2 arms with 2 segments each, 2 legs with 2 segments each). Define the joint connections and basic geometric shapes for each link.