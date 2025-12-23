# URDF for Humanoid Robots - Summary

**Quick Reference**: Key concepts from this lesson

## Core Concept

URDF (Unified Robot Description Format) is an XML-based format for describing robot physical structure through links (rigid body parts) and joints (connections with motion constraints), creating kinematic trees that enable simulation, visualization, and motion planning.

## Key Points

- **Links Represent Body Parts**: Links define rigid bodies (torso, arms, legs, sensors) with visual geometry (cylinders, boxes, meshes), collision shapes, and inertial properties. Each link has a name and coordinate frame.

- **Joints Connect Links with Motion Constraints**: Joints specify parent-child relationships and motion type (revolute for rotation, prismatic for sliding, fixed for rigid connection). Each joint has limits (angle/distance ranges), axis of motion, and effort/velocity constraints.

- **Kinematic Trees Define Robot Structure**: URDF robots form hierarchical trees with one root link and parent-child relationships through joints. Moving a parent link moves all descendant links—crucial for forward/inverse kinematics.

- **Standard Units and Conventions**: URDF requires meters (distance), radians (angles), kilograms (mass). Coordinate frames use right-hand rule (X=forward, Y=left, Z=up). Getting units wrong causes visualization/simulation errors.

- **Enables Simulation and Visualization**: Gazebo reads URDF for physics simulation (testing control safely). RViz reads URDF for 3D visualization (debugging in real-time). MoveIt reads URDF for motion planning (computing trajectories).

## When to Use

**URDF is essential for**:
- **Simulation**: Testing robot behaviors in Gazebo before deploying to hardware
- **Visualization**: Debugging robot state and movements in RViz
- **Motion Planning**: Enabling libraries like MoveIt to compute collision-free trajectories
- **Standardization**: Sharing robot models across teams and tools
- **Hardware Abstraction**: Developing algorithms independent of specific actuators

## Common Patterns

- **Root link** (base_link or torso) serves as the kinematic tree root for mobile/humanoid robots
- **Revolute joints** for rotating connections (shoulders, elbows, knees) with angle limits
- **Visual geometry** uses simple shapes (cylinder, box) for limbs; complex meshes (STL, DAE) for detailed models
- **Origin tags** position joints relative to parent link frames using xyz (meters) and rpy (radians)
- **Joint limits** constrain motion to realistic ranges based on physical hardware or biological analogs

## Related Concepts

- **Previous Lessons**: ROS2 Fundamentals (Lesson 1), Communication Patterns (Lesson 2), Python rclpy (Lesson 3)—URDF completes the foundational knowledge
- **Next Steps**: Capstone Project integrating all concepts (multi-node ROS2 system + URDF model)
- **Advanced Topics**: Xacro (XML macros for parameterized URDF), SDF (Simulation Description Format), mesh generation, sensor integration
