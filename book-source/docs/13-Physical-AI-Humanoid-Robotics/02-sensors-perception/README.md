# Module 2: Sensors and Perception for Humanoid Robots

**Focus**: How humanoid robots sense and understand their environment

This module builds on Module 1 (ROS 2 fundamentals) to explore how humanoid robots perceive the world through multiple sensor modalities. You'll learn about cameras, depth sensors, IMUs, and how to combine them through sensor fusion for robust perception.

## Learning Goals

- Understand camera systems and computer vision basics for humanoid robots
- Learn LiDAR and depth sensing technologies
- Master IMU (Inertial Measurement Unit) for balance and orientation
- Apply sensor fusion techniques to combine multiple sensor inputs

## Prerequisites

- **Module 1 Complete**: Understanding of ROS 2 nodes, topics, publishers, subscribers, and message types
- **Python 3.11+**: Object-oriented programming, type hints, imports
- **Basic Linear Algebra**: Vectors, matrices, transformations (for sensor fusion)

## Module Structure

### [Lesson 1: Camera Systems and Computer Vision](./01-camera-systems.md)
**Time**: 30-45 minutes | **Level**: Beginner

Learn how humanoid robots use cameras to perceive their environment, including camera types (monocular, stereo, RGB-D), ROS 2 Image messages, and camera placement strategies for humanoid tasks.

**Key Concepts**: Camera types, sensor_msgs/Image, field of view, resolution

---

### [Lesson 2: Depth Sensing Technologies](./02-depth-sensing.md)
**Time**: 30-45 minutes | **Level**: Beginner-Intermediate

Explore how humanoid robots measure distances using LiDAR, structured light cameras, and time-of-flight sensors. Understand point clouds and depth data representation in ROS 2.

**Key Concepts**: LiDAR, depth cameras, sensor_msgs/PointCloud2, sensor_msgs/LaserScan

---

### [Lesson 3: IMU and Proprioception](./03-imu-proprioception.md)
**Time**: 30-45 minutes | **Level**: Intermediate

Understand how humanoid robots maintain balance and know their body position using Inertial Measurement Units (IMUs). Learn about accelerometers, gyroscopes, magnetometers, and proprioception.

**Key Concepts**: IMU components, sensor_msgs/Imu, drift mitigation, balance control

---

### [Lesson 4: Sensor Fusion Techniques](./04-sensor-fusion.md)
**Time**: 30-45 minutes | **Level**: Intermediate

Discover how humanoid robots combine data from multiple sensors (cameras, LiDAR, IMU) to create robust perception. Learn Kalman filtering, complementary filtering, and visual-inertial odometry concepts.

**Key Concepts**: Sensor fusion, Kalman filter, visual-inertial odometry, robot_localization

---

### [Capstone Project: Multi-Sensor Perception System Design](./05-capstone-project.md)
**Time**: 60-90 minutes | **Level**: Intermediate

Apply your knowledge from all 4 lessons to design a multi-sensor perception system for a humanoid robot task. Select sensors, justify fusion strategies, and sketch ROS 2 node architecture.

**Integration**: Cameras + Depth Sensors + IMU + Sensor Fusion

---

### [Quiz: Module 2 Assessment](./06-quiz.md)
**Time**: 15-20 minutes | **Questions**: 15-20

Test your understanding of sensor perception concepts across all lessons. Includes conceptual questions, code-reading exercises, and scenario-based problems.

---

## Learning Approach

Each lesson follows a consistent structure:

- **💬 AI Colearning Prompts**: Explore concepts with AI assistants (Claude, ChatGPT)
- **🎓 Expert Insights**: Learn best practices and common pitfalls
- **🤝 Practice Exercises**: Apply concepts through design challenges
- **Real-World Examples**: Case studies from Boston Dynamics, Tesla Optimus, Agility Robotics

## Hands-On Practice

While this module focuses on conceptual understanding, you can enhance learning with:

- **RViz Visualization**: Visualize sensor data (camera images, point clouds, IMU axes)
- **Gazebo Simulation**: Generate synthetic sensor data without hardware
- **ROS 2 Tools**: Use `ros2 topic echo` to inspect sensor messages

## Success Criteria

After completing this module, you should be able to:

- ✅ Differentiate between monocular, stereo, and RGB-D cameras
- ✅ Explain LiDAR, structured light, and time-of-flight depth sensing
- ✅ Describe IMU components and their role in balance control
- ✅ Design a sensor fusion strategy for a humanoid robot task
- ✅ Score 80%+ on the module quiz

## Next Module

After mastering sensor perception, you'll be ready for:

**Module 3**: Motion Planning and Control (coming soon)
Learn how humanoid robots plan collision-free paths and execute stable locomotion using sensor feedback.

---

**Estimated Total Time**: 3-4 hours (including capstone and quiz)
**Target Audience**: CS students with Python + ROS 2 Module 1 knowledge
