---
id: capstone-project
title: "Capstone Project: Design a Delivery Robot System"
sidebar_position: 5
description: "Integrative project combining ROS2 architecture, node communication patterns, Python rclpy code, and URDF robot modeling to design a complete autonomous delivery robot system."
time_estimate: "120 minutes"
difficulty_level: "intermediate"
prerequisites: ["01-ros2-fundamentals", "02-nodes-topics-services", "03-python-rclpy-bridge", "04-urdf-humanoid-basics"]
related_lessons: ["01-ros2-fundamentals", "02-nodes-topics-services", "03-python-rclpy-bridge", "04-urdf-humanoid-basics"]
assessment_method: "capstone project"
skills:
  - name: "ROS2 System Architecture Design"
    proficiency_level: "beginner"
    category: "system-design"
    bloom_level: "apply"
    digcomp_area: "problem-solving"
    measurable_at_this_level: "design multi-node ROS2 system with appropriate communication patterns"
  - name: "Node Communication Pattern Selection"
    proficiency_level: "beginner"
    category: "system-design"
    bloom_level: "apply"
    digcomp_area: "problem-solving"
    measurable_at_this_level: "choose topics vs services for different robot scenarios and justify selections"
  - name: "rclpy Code Structure"
    proficiency_level: "beginner"
    category: "programming"
    bloom_level: "apply"
    digcomp_area: "programming"
    measurable_at_this_level: "outline Python rclpy code for nodes with publishers/subscribers"
  - name: "URDF Robot Modeling"
    proficiency_level: "beginner"
    category: "robot-modeling"
    bloom_level: "apply"
    digcomp_area: "technical-concepts"
    measurable_at_this_level: "design URDF structure for robot with links and joints"
learning_objectives:
  - objective: "Integrate ROS2 concepts (nodes, topics, services) into a cohesive system design"
    proficiency_level: "beginner"
    bloom_level: "apply"
    assessment_method: "capstone deliverable - node diagram and communication architecture"
  - objective: "Apply communication pattern selection (topics vs services) to real robot scenarios"
    proficiency_level: "beginner"
    bloom_level: "apply"
    assessment_method: "capstone deliverable - topic/service design with justifications"
  - objective: "Outline rclpy code structure for multi-node system"
    proficiency_level: "beginner"
    bloom_level: "apply"
    assessment_method: "capstone deliverable - Python code outlines with key components"
  - objective: "Design URDF robot model with appropriate links and joints"
    proficiency_level: "beginner"
    bloom_level: "apply"
    assessment_method: "capstone deliverable - URDF structure diagram or pseudocode"
cognitive_load:
  new_concepts: 0
  assessment: "high - integrates all 4 lessons, requires synthesis and application of multiple concepts simultaneously"
differentiation:
  extension_for_advanced: "Implement the delivery robot system in actual ROS2 code. Create full URDF file and test in Gazebo simulator. Add advanced features like obstacle avoidance or multi-robot coordination."
  remedial_for_struggling: "Complete the project in stages: first node diagram only, then add communication patterns, then rclpy outlines, then URDF. Review individual lessons as needed."
tags: ["ros2", "capstone", "project", "integration", "system-design", "rclpy", "urdf", "delivery-robot"]
generated_by: "manual"
created: "2025-12-06"
last_modified: "2025-12-06"
ros2_version: "humble"
---

# Capstone Project: Design a Delivery Robot System

Integrate everything you've learned—ROS2 architecture, node communication, Python rclpy, and URDF modeling—into a complete autonomous delivery robot system design.

## Project Overview

You're tasked with designing an **autonomous delivery robot for indoor environments** (hospitals, offices, warehouses). This robot must navigate between locations, avoid obstacles, accept delivery requests, and report its status. Your goal is to create a complete system design that demonstrates your understanding of ROS2 fundamentals, not to write production code.

**The Scenario**: A hospital needs autonomous robots to deliver medications, lab samples, and supplies between departments. The robot operates in hallways with people, doors, and obstacles. It receives delivery requests from a central scheduling system, navigates to the pickup location, confirms the item is loaded, travels to the destination, and notifies staff when it arrives.

**What You'll Design**:
1. **ROS2 Node Architecture**: Which nodes does your system need, and what is each responsible for?
2. **Communication Patterns**: Which topics and services connect your nodes, and why did you choose each pattern?
3. **rclpy Code Outlines**: Pseudocode or skeleton Python code showing key node structure
4. **URDF Robot Model**: Physical structure of the delivery robot (links, joints, sensors)

This capstone integrates all four lessons. It's conceptual—you're designing the architecture, not implementing every line of code. Think like a robotics system architect: what components are needed, how do they communicate, and why is this architecture appropriate?

## Project Requirements

Your delivery robot system design must include:

### 1. Node Architecture Diagram (25 points)

Create a diagram showing at least **4-6 nodes** in your system. Each node should have a clear, single responsibility.

**Required nodes (examples)**:
- **Navigation Node**: Plans paths from current location to destination
- **Obstacle Detection Node**: Processes sensor data (LIDAR, cameras) to detect obstacles
- **Motor Control Node**: Commands wheel motors based on navigation instructions
- **Task Manager Node**: Receives delivery requests, coordinates the workflow
- **Battery Monitor Node**: Tracks battery level, triggers charging behavior
- **Status Publisher Node**: Broadcasts robot state for monitoring dashboards

For each node, specify:
- **Name** (descriptive, like `navigation_planner` or `obstacle_detector`)
- **Responsibility** (one sentence describing what it does)
- **ROS2 graph position** (which other nodes does it connect to?)

### 2. Communication Design (25 points)

For each connection between nodes, specify whether it's a **topic** (pub/sub) or **service** (request/response) and **justify your choice**.

**Required communications (minimum 5-7)**:
- Sensor data flow (LIDAR → obstacle detection)
- Navigation commands (planner → motor control)
- Delivery requests (scheduler → task manager)
- Status updates (various nodes → monitoring)
- Emergency stop (safety system → motors)

For each communication, provide:
- **Name** (like `/scan`, `/cmd_vel`, `/request_delivery`)
- **Message type** (example: `sensor_msgs/LaserScan`, `geometry_msgs/Twist`, custom message)
- **Pattern** (topic or service)
- **Justification** (why this pattern? Consider: continuous vs discrete, acknowledgment needed, frequency, reliability)

**Example**:
- `/scan` topic (sensor_msgs/LaserScan): LIDAR publishes at 10Hz, multiple subscribers (obstacle detection, mapping). Topic chosen because continuous data stream, many-to-many.
- `/request_delivery` service (custom DeliveryRequest/DeliveryResponse): Scheduler requests delivery, waits for confirmation. Service chosen because discrete request requiring acknowledgment.

### 3. rclpy Code Outlines (25 points)

Write **pseudocode or skeleton Python code** for at least **2-3 nodes** demonstrating rclpy structure.

Your code outlines should show:
- Node class inheriting from `rclpy.node.Node`
- Publishers and subscribers created in `__init__()`
- Callback functions for message processing
- Timer setup if periodic behavior needed
- Type hints for clarity

**You don't need to implement full logic**—focus on demonstrating rclpy patterns (publishers, subscribers, callbacks, timers).

**Example skeleton**:

```python
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleDetector(Node):
    def __init__(self) -> None:
        super().__init__('obstacle_detector')

        # Subscribe to LIDAR data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # Publish obstacle alerts
        self.alert_pub = self.create_publisher(
            # Custom message type for obstacle alerts
            # ...
        )

        self.get_logger().info('Obstacle detector started')

    def lidar_callback(self, msg: LaserScan) -> None:
        # Process scan data, detect obstacles
        # If obstacle detected within threshold, publish alert
        pass
```

### 4. URDF Robot Structure (25 points)

Design the physical structure of your delivery robot using URDF concepts (links, joints).

Your robot should include:
- **Base/chassis** (root link)
- **Wheels** (at least 2, consider differential drive or omnidirectional)
- **Sensor mounts** (LIDAR, cameras)
- **Cargo compartment** (for deliveries)
- **Optional**: Manipulator arm for pickup/dropoff

Specify:
- **Links**: Name, approximate dimensions (meters), purpose
- **Joints**: Name, type (revolute, continuous, fixed), parent-child relationships
- **Kinematic tree**: Show parent-child hierarchy

You can present this as:
- Sketch/diagram with annotations
- URDF pseudocode showing key `<link>` and `<joint>` tags
- Hierarchical list describing the structure

**Example structure**:
```
base_link (40cm x 30cm x 20cm chassis)
├── left_wheel (via left_wheel_joint, continuous rotation)
├── right_wheel (via right_wheel_joint, continuous rotation)
├── lidar_mount (via lidar_joint, fixed, 30cm above base)
│   └── lidar_sensor (fixed)
└── cargo_compartment (via cargo_joint, fixed, rear of base)
```

## Design Guidance

### Hints for Success

**Start with the Mission**: What does the robot need to accomplish? List the capabilities (navigate, avoid obstacles, accept deliveries, report status). Each capability suggests nodes.

**One Node, One Job**: Don't create a monolithic "robot controller" node. Break responsibilities into focused modules. Navigation planning is separate from motor control, which is separate from obstacle detection. This modularity makes the system testable and maintainable.

**Topics for Data Streams, Services for Commands**: Sensor data flows continuously (topics). Delivery requests are discrete transactions requiring confirmation (services). Emergency stops must be reliable (topic with reliable QoS). When in doubt, ask: "Is this continuous or one-time? Does the sender need confirmation?"

**Real Robots Are Messy**: Don't worry about perfection. Real robot systems evolve. Your design might have 5 nodes or 15—both can be valid. The key is justifying your choices: "I chose a topic here because..." or "I separated these nodes because..."

**URDF Is About Structure, Not Behavior**: Your URDF describes what the robot looks like (links, joints, dimensions), not what it does. A wheel joint is continuous (rotates freely), but the URDF doesn't specify how fast or when—that's the motor controller's job.

### Example Structure (Not the Only Answer!)

Here's **one possible approach** to get you thinking. Your design will differ based on your assumptions and priorities.

**Possible Node Structure**:
- `task_manager`: Receives delivery requests, coordinates workflow (request pickup → navigate → confirm → navigate → notify)
- `navigation_planner`: Computes paths from current pose to goal pose
- `localization`: Tracks robot position using wheel encoders + LIDAR
- `obstacle_avoidance`: Processes LIDAR scans, adjusts path if obstacles detected
- `motor_controller`: Commands wheel velocities based on navigation plan
- `battery_monitor`: Publishes battery state, triggers low-battery behavior
- `status_publisher`: Aggregates robot state for monitoring dashboards

**Communication examples**:
- `/scan` topic: LIDAR → obstacle_avoidance, localization (continuous sensor data)
- `/cmd_vel` topic: navigation_planner → motor_controller (continuous velocity commands)
- `/odom` topic: motor_controller → localization (wheel odometry)
- `/robot_status` topic: various → monitoring (periodic status updates)
- `/request_delivery` service: scheduler → task_manager (discrete delivery request)
- `/confirm_pickup` service: task_manager → operator interface (confirm item loaded)

**URDF example**:
- base_link (30cm x 40cm x 15cm)
- 2 drive wheels (continuous joints) + 1 caster wheel (fixed spherical)
- LIDAR sensor mount (fixed, 25cm above base for 360° view)
- Cargo bin (fixed to rear of base, 20cm x 20cm x 15cm)

This is just **one** design. A valid alternative might use 4 omnidirectional wheels, separate localization and mapping nodes, or a robotic arm for autonomous pickup. The key is **justifying your choices** based on ROS2 principles from Lessons 1-4.

## Evaluation Rubric

Your capstone project will be evaluated on how well you integrate and apply concepts from all four lessons.

| **Criterion** | **Exemplary (23-25 pts)** | **Proficient (18-22 pts)** | **Developing (13-17 pts)** | **Needs Work (0-12 pts)** |
|---------------|---------------------------|----------------------------|----------------------------|---------------------------|
| **Node Architecture** | 5-7 nodes with clear, single responsibilities. Logical separation of concerns (sensing, planning, control). ROS2 graph shows appropriate modularity. | 4-5 nodes with mostly clear responsibilities. Minor overlap or vague descriptions. Reasonable modularity. | 2-3 nodes or unclear responsibilities. Significant overlap (monolithic nodes) or missing key components. | Fewer than 2 nodes or unclear architecture. Doesn't demonstrate understanding of modular node design. |
| **Communication Design** | 6-8 communications with correct pattern selection (topics vs services). Clear justifications based on data flow, timing, and reliability. Demonstrates understanding of pub/sub vs request/response. | 4-5 communications with mostly correct patterns. Justifications present but may lack depth. Minor pattern mismatches (e.g., using service for continuous data). | 2-3 communications or several incorrect patterns. Weak or missing justifications. Doesn't clearly apply topics vs services principles. | Fewer than 2 communications or fundamentally incorrect patterns. No justifications or misunderstands pub/sub and services. |
| **rclpy Code Structure** | 2-3 nodes with clear code outlines showing Node inheritance, publishers/subscribers, callbacks, timers. Type hints used. Demonstrates rclpy patterns from Lesson 3. | 1-2 nodes with code outlines showing basic structure. Minor omissions (missing callbacks or unclear structure). Demonstrates some rclpy understanding. | 1 node with incomplete code outline or significant structural issues. Doesn't clearly show publishers/subscribers or callbacks. | No code outlines or incorrect Python structure. Doesn't demonstrate rclpy understanding. |
| **URDF Robot Structure** | Complete robot structure with 5-8 links, appropriate joints (revolute, continuous, fixed). Clear kinematic tree. Dimensions and sensor placement realistic. Demonstrates URDF concepts from Lesson 4. | Basic robot structure with 3-4 links and joints. Kinematic tree present but may lack detail. Reasonable dimensions. Shows understanding of links and joints. | Incomplete robot structure (1-2 links) or unclear joints. Missing kinematic tree or unrealistic design. Limited URDF understanding. | No URDF structure or incorrect concepts. Doesn't demonstrate understanding of links, joints, or kinematic trees. |
| **Integration & Justification** | Design integrates all 4 lessons cohesively. Choices justified with references to ROS2 principles (modularity, communication patterns, ecosystem). Shows systems thinking. | Design addresses most lessons with reasonable integration. Some justifications present. Shows understanding of how concepts connect. | Design addresses 2-3 lessons with weak integration. Few justifications or doesn't connect concepts well. | Design addresses fewer than 2 lessons or shows no integration. No justifications or understanding of how ROS2 concepts work together. |

**Total: 100 points** (5 criteria × 25 points each)

**Passing**: 70+ points demonstrates sufficient understanding to proceed to advanced robotics topics.

**Exemplary**: 90+ points indicates strong mastery of ROS2 fundamentals and readiness for complex system design.

## Optional: Hands-On Implementation

This capstone is designed as a **design exercise** you can complete with pen/paper, diagramming tools, or a text editor. However, if you want to go further and implement your design in actual ROS2 code, here's how to get started.

### Setup for Hands-On Execution

**Prerequisites**:
- Ubuntu 22.04 (native or WSL2 on Windows)
- ROS2 Humble installed ([installation guide](https://docs.ros.org/en/humble/Installation.html))
- Python 3.10+
- Text editor or IDE (VS Code recommended)

**Quick Start**:
1. Create ROS2 workspace: `mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src`
2. Create Python package: `ros2 pkg create --build-type ament_python delivery_robot --dependencies rclpy std_msgs sensor_msgs geometry_msgs`
3. Write your node code in `delivery_robot/delivery_robot/`
4. Build: `cd ~/ros2_ws && colcon build`
5. Source: `source install/setup.bash`
6. Run nodes: `ros2 run delivery_robot <node_name>`

**Testing**:
- Publish test data: `ros2 topic pub /scan sensor_msgs/LaserScan "{...}"`
- Monitor topics: `ros2 topic echo /cmd_vel`
- Visualize graph: `rqt_graph`
- Visualize URDF: `ros2 launch urdf_tutorial display.launch.py model:=robot.urdf`

**Resources**:
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [rclpy API Reference](https://docs.ros2.org/humble/api/rclpy/)
- [URDF Tutorial](http://wiki.ros.org/urdf/Tutorials)
- [Gazebo Simulation](https://gazebosim.org/docs)

**Note**: Hands-on implementation is **optional** and **not required** for capstone completion. The design exercise alone demonstrates your understanding. If you choose to implement, expect 10-20 additional hours for coding, debugging, and testing.

## Submission Guidelines

Your capstone deliverable should include:

1. **Node Architecture Diagram**: Visual diagram or clear text description of nodes and their responsibilities
2. **Communication Design**: Table or list of topics/services with names, types, patterns, and justifications
3. **rclpy Code Outlines**: Pseudocode or skeleton code for 2-3 nodes (can be in Markdown code blocks or Python files)
4. **URDF Robot Structure**: Diagram, URDF pseudocode, or hierarchical description of robot links and joints
5. **Design Rationale** (optional): 1-2 paragraphs explaining your overall architecture choices and how they address the delivery robot requirements

**Format**: Submit as a single Markdown document, PDF, or GitHub repository README. Include your name and date.

**Length**: Expect 3-6 pages depending on detail level and formatting.

**Time**: Allocate 2-3 hours for design and documentation.

## Summary

This capstone project integrates **ROS2 fundamentals** (Lesson 1), **node communication patterns** (Lesson 2), **Python rclpy programming** (Lesson 3), and **URDF robot modeling** (Lesson 4) into a cohesive system design. By completing this project, you demonstrate your ability to:

- Design modular ROS2 architectures with appropriate node separation
- Select communication patterns (topics vs services) based on data flow and timing requirements
- Structure rclpy code with publishers, subscribers, callbacks, and timers
- Model robot physical structure using URDF links, joints, and kinematic trees
- Integrate multiple concepts into a practical robotic system

This is **systems thinking**—not just understanding individual concepts but knowing how to combine them to solve real problems. The delivery robot scenario is representative of actual robotics challenges: coordinating sensors, planning, control, and task management in a distributed system.

Completing this capstone prepares you for advanced robotics topics like motion planning (MoveIt), SLAM (simultaneous localization and mapping), computer vision integration, and autonomous navigation. You've built the foundation; now you're ready to construct increasingly sophisticated robotic behaviors.

**Congratulations on completing Module 1: The Robotic Nervous System (ROS 2)!** You now have the fundamental knowledge to understand how modern robots are built, communicate, and operate. Keep building, keep learning, and welcome to the robotics community.
