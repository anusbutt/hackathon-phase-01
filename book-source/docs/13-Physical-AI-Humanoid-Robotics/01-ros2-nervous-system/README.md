# Module 1: The Robotic Nervous System (ROS 2)

**Focus**: Middleware for robot control
**Estimated Time**: 4 lessons × 30-45 minutes each = 2-3 hours
**Level**: Beginner

## Overview

Just as the human nervous system coordinates signals between the brain, sensors, and muscles, ROS 2 (Robot Operating System 2) serves as the communication middleware that enables different parts of a robot to work together seamlessly. This module introduces you to ROS 2 as the foundational software layer for humanoid robotics.

## Learning Goals

By the end of this module, you will:

- **Understand** what ROS 2 is and why it's essential for robot control
- **Explain** how nodes, topics, and services enable distributed robot systems
- **Read and interpret** Python code using rclpy to create ROS 2 nodes
- **Describe** how URDF represents humanoid robot structures for simulation and control

## Module Structure

### [Lesson 1: ROS2 Fundamentals](./01-ros2-fundamentals.md)
**Time**: 30-45 minutes | **Level**: Beginner

Learn what ROS 2 is, its role as middleware, and the pub/sub communication model. Discover why ROS 2 uses DDS for real-time robot communication.

### [Lesson 2: Nodes, Topics, and Services](./02-nodes-topics-services.md)
**Time**: 30-45 minutes | **Level**: Beginner

Understand how different robot components communicate using ROS 2 patterns. Explore the ROS 2 graph, Quality of Service (QoS), and when to use topics vs services.

### [Lesson 3: Python rclpy Bridge](./03-python-rclpy-bridge.md)
**Time**: 30-45 minutes | **Level**: Beginner

Connect your Python programming knowledge to ROS 2 using the rclpy library. Learn the Node class, publishers, subscribers, callbacks, and timers through practical code examples.

### [Lesson 4: URDF for Humanoid Robots](./04-urdf-humanoid-basics.md)
**Time**: 30-45 minutes | **Level**: Beginner

Discover how robots describe their physical structure using URDF (Unified Robot Description Format). Understand links, joints, and kinematic trees for humanoid robot modeling.

### [Capstone Project: Design a Delivery Robot System](./05-capstone-project.md)
**Time**: 2-3 hours | **Level**: Beginner

Design a complete autonomous delivery robot system integrating all four lesson concepts. Submit node architecture diagrams, communication patterns, rclpy code outlines, and URDF structure with justifications.

### [Module Quiz](./06-quiz.md)
**Time**: 30-45 minutes | **15 Questions** | **Passing**: 12/18 points (67%)

Test your understanding with questions covering all lessons, including multiple choice, short answer, and code reading exercises.

## Prerequisites

- **Python**: OOP, type hints, imports, basic async concepts
- **Networking**: Basic understanding of client-server, pub/sub patterns
- **No ROS 2 installation required**: This module focuses on conceptual understanding

## How to Use This Module

1. **Sequential Learning**: Complete lessons 1-4 in order, as each builds on previous concepts
2. **AI Colearning**: Use the 💬 prompts to explore concepts with Claude or ChatGPT
3. **Practice Exercises**: Try the 🤝 challenges to apply what you've learned
4. **Capstone Integration**: After all lessons, complete the capstone to synthesize knowledge

## Success Criteria

You'll know you've mastered this module when you can:

- Explain ROS 2's purpose to a peer or AI assistant
- Identify appropriate communication patterns for robot scenarios
- Read and understand rclpy code conceptually
- Interpret URDF descriptions of robot structures

## Next Steps

Ready to begin? Start with [Lesson 1: ROS2 Fundamentals](./01-ros2-fundamentals.md) to understand what ROS 2 is and why every roboticist needs to know it.
