---
id: quiz
title: "Module 1 Quiz: The Robotic Nervous System (ROS 2)"
sidebar_position: 6
description: "Assessment quiz covering ROS2 fundamentals, node communication, Python rclpy, and URDF robot modeling. 15 questions testing comprehension across all four lessons. Passing score: 12/18 points (67%)."
time_estimate: "45 minutes"
difficulty_level: "beginner"
prerequisites: ["01-ros2-fundamentals", "02-nodes-topics-services", "03-python-rclpy-bridge", "04-urdf-humanoid-basics"]
related_lessons: ["01-ros2-fundamentals", "02-nodes-topics-services", "03-python-rclpy-bridge", "04-urdf-humanoid-basics", "05-capstone-project"]
assessment_method: "quiz"
skills:
  - name: "ROS2 Comprehension"
    proficiency_level: "beginner"
    category: "knowledge-assessment"
    bloom_level: "understand"
    digcomp_area: "technical-concepts"
    measurable_at_this_level: "demonstrate understanding of ROS2 concepts through quiz performance (12/18 points minimum)"
  - name: "Code Reading (rclpy and URDF)"
    proficiency_level: "beginner"
    category: "programming"
    bloom_level: "understand"
    digcomp_area: "programming"
    measurable_at_this_level: "read and explain Python rclpy code and URDF XML snippets"
learning_objectives:
  - objective: "Assess understanding of ROS2 fundamentals, DDS, and modularity"
    proficiency_level: "beginner"
    bloom_level: "understand"
    assessment_method: "questions 1-4 (Lesson 1)"
  - objective: "Assess understanding of node communication patterns (topics vs services, QoS)"
    proficiency_level: "beginner"
    bloom_level: "apply"
    assessment_method: "questions 5-8 (Lesson 2)"
  - objective: "Assess ability to read and explain rclpy Python code"
    proficiency_level: "beginner"
    bloom_level: "understand"
    assessment_method: "questions 9-11 (Lesson 3)"
  - objective: "Assess ability to read and interpret URDF XML code"
    proficiency_level: "beginner"
    bloom_level: "understand"
    assessment_method: "questions 12-13 (Lesson 4)"
  - objective: "Assess ability to integrate concepts across multiple lessons"
    proficiency_level: "beginner"
    bloom_level: "apply"
    assessment_method: "questions 14-15 (integration)"
cognitive_load:
  new_concepts: 0
  assessment: "assessment tool - no new concepts introduced, evaluates retention and comprehension from Lessons 1-4"
differentiation:
  extension_for_advanced: "Challenge: Score 16+/18 points (89%+). Then complete hands-on implementation of quiz code examples in actual ROS2 environment."
  remedial_for_struggling: "If scoring below 12/18, review specific lessons indicated in study guide, focus on one lesson at a time, then retake quiz."
tags: ["quiz", "assessment", "ros2", "rclpy", "urdf", "evaluation"]
generated_by: "manual"
created: "2025-12-06"
last_modified: "2025-12-06"
ros2_version: "humble"
---

# Module 1 Quiz: The Robotic Nervous System (ROS 2)

Test your understanding of ROS2 fundamentals, node communication patterns, Python rclpy programming, and URDF robot modeling.

## Instructions

- **Total Questions**: 15
- **Passing Score**: 12/15 correct (80%)
- **Time Limit**: 45 minutes (recommended)
- **Question Types**: Multiple choice, short answer, code reading
- **Resources**: You may refer to lesson notes during the quiz
- **Scoring**:
  - Multiple choice: 1 point each
  - Short answer: 1-2 points each (partial credit available)
  - Code reading: 2 points each (with rubric)

**Need to review?** Questions reference specific lessons. If you're unsure, revisit:
- Questions 1-4: [Lesson 1 - ROS2 Fundamentals](./ros2-fundamentals)
- Questions 5-8: [Lesson 2 - Nodes, Topics, and Services](./nodes-topics-services)
- Questions 9-11: [Lesson 3 - Python rclpy Bridge](./python-rclpy-bridge)
- Questions 12-13: [Lesson 4 - URDF for Humanoid Robots](./urdf-humanoid-basics)
- Questions 14-15: Integration across multiple lessons

---

## Questions

### Lesson 1: ROS2 Fundamentals (Questions 1-4)

**Question 1 (Multiple Choice)** - 1 point

What is the primary role of ROS2 in a robotic system?

A) Operating system that replaces Linux on robot computers
B) Middleware that enables distributed communication between robot components
C) Programming language for writing robot control code
D) Hardware driver for sensors and motors

<details>
<summary>Reveal Answer</summary>

**Correct Answer: B**

**Explanation**: ROS2 is middleware (not an OS or programming language) that provides the communication layer enabling robot components to work together as a distributed system. It runs on top of operating systems like Linux and uses languages like Python and C++.

*Lesson Reference: 01-ros2-fundamentals.md (What Is ROS2?)*
</details>

---

**Question 2 (Multiple Choice)** - 1 point

Which underlying communication protocol does ROS2 use, making it suitable for real-time and mission-critical applications?

A) HTTP/REST
B) WebSockets
C) Data Distribution Service (DDS)
D) MQTT

<details>
<summary>Reveal Answer</summary>

**Correct Answer: C**

**Explanation**: ROS2 uses DDS (Data Distribution Service), an industry-standard middleware used in air traffic control and military systems. DDS provides real-time capabilities, handles network disruptions gracefully, and enables peer-to-peer node discovery without a central master.

*Lesson Reference: 01-ros2-fundamentals.md (What Is ROS2?, Key Principles)*
</details>

---

**Question 3 (Short Answer)** - 2 points

Explain in 2-3 sentences why modularity is important in ROS2 robot systems. Provide one specific example of how modularity benefits development or debugging.

<details>
<summary>Reveal Answer & Rubric</summary>

**Sample Answer**: Modularity allows robot capabilities to be developed as independent nodes with single responsibilities, making systems testable and maintainable. For example, if the vision processing node crashes, other nodes (motor control, balance) continue running, preventing total system failure. You can also develop, test, and upgrade nodes separately without affecting the entire system.

**Grading Rubric (2 points total)**:
- **2 points**: Explains modularity (independent components with single responsibilities) AND provides specific, correct example of benefits (testing, fault isolation, parallel development, upgradeability)
- **1 point**: Explains modularity concept OR provides example, but not both, or explanation lacks clarity
- **0 points**: Incorrect explanation or irrelevant example

*Lesson Reference: 01-ros2-fundamentals.md (Why ROS2 Matters for Physical AI)*
</details>

---

**Question 4 (Short Answer)** - 1 point

What is the "ROS2 graph"? In one sentence, describe what it represents.

<details>
<summary>Reveal Answer & Rubric</summary>

**Sample Answer**: The ROS2 graph is a living map of all running nodes and their topic connections, showing the real-time structure of the robot's software architecture.

**Grading Rubric (1 point)**:
- **1 point**: Correctly identifies graph as visualization/representation of nodes AND their connections/topics
- **0 points**: Incorrect or incomplete (e.g., mentions only nodes or only topics, not both)

*Lesson Reference: 01-ros2-fundamentals.md (Key Principles #3)*
</details>

---

### Lesson 2: Nodes, Topics, and Services (Questions 5-8)

**Question 5 (Multiple Choice)** - 1 point

When should you use a **topic** (publish/subscribe) instead of a **service** (request/response)?

A) When you need confirmation that a command was executed
B) When data flows continuously and multiple subscribers may need it
C) When only one node needs to receive the data
D) When the requester must wait for the operation to complete

<details>
<summary>Reveal Answer</summary>

**Correct Answer: B**

**Explanation**: Topics are for continuous data streams (like sensor data) that flow asynchronously to potentially multiple subscribers. Services are for discrete requests requiring confirmation/response. Topics don't block the publisher and support many-to-many communication.

*Lesson Reference: 02-nodes-topics-services.md (Key Principles - Topics vs Services)*
</details>

---

**Question 6 (Multiple Choice)** - 1 point

What happens when a ROS2 node publishes a message to a topic, but there are no subscribers listening?

A) The node crashes with an error
B) The message is queued indefinitely until a subscriber connects
C) The message is broadcast normally and discarded (no error)
D) ROS2 sends a warning to all other nodes

<details>
<summary>Reveal Answer</summary>

**Correct Answer: C**

**Explanation**: Publishers broadcast messages without knowing who (if anyone) is listening. If no subscribers exist, the message is simply not received—this is the "loose coupling" advantage of pub/sub. No error occurs because publishers and subscribers are decoupled.

*Lesson Reference: 02-nodes-topics-services.md (Key Principles - Topics)*
</details>

---

**Question 7 (Multiple Choice)** - 1 point

Quality of Service (QoS) policies in ROS2 allow you to tune which tradeoff?

A) Code readability vs performance
B) Reliability vs latency
C) Memory usage vs CPU usage
D) Python vs C++ execution speed

<details>
<summary>Reveal Answer</summary>

**Correct Answer: B**

**Explanation**: QoS policies control reliability vs latency tradeoffs. "Reliable" delivery guarantees messages arrive (like TCP) but adds latency. "Best effort" delivery tolerates occasional loss for lower latency—appropriate for high-frequency sensor data where the next reading is more valuable than a delayed old one.

*Lesson Reference: 02-nodes-topics-services.md (Key Principles - QoS Policies, Expert Insight)*
</details>

---

**Question 8 (Short Answer)** - 1 point

Give one example of a robot scenario where you would use a **service** instead of a topic. Explain why a service is more appropriate than a topic for this scenario.

<details>
<summary>Reveal Answer & Rubric</summary>

**Sample Answer**: Requesting a robot to move to a specific position. You use a service because you need confirmation that the robot reached the target before proceeding to the next task, and it's a discrete one-time command (not continuous data).

**Other valid examples**: Triggering calibration, querying current robot state, confirming item pickup, requesting collision check.

**Grading Rubric (1 point)**:
- **1 point**: Valid scenario AND correct justification (needs confirmation/acknowledgment, discrete request, synchronous behavior needed)
- **0 points**: Invalid scenario (should use topic) or missing/incorrect justification

*Lesson Reference: 02-nodes-topics-services.md (Practical Example, Why Node Communication Matters)*
</details>

---

### Lesson 3: Python rclpy Bridge (Questions 9-11)

**Question 9 (Multiple Choice)** - 1 point

In rclpy, what does `rclpy.spin(node)` do?

A) Rotates the robot physically
B) Runs the event loop to process callbacks and keep the node alive
C) Initializes the ROS2 middleware
D) Creates a new publisher

<details>
<summary>Reveal Answer</summary>

**Correct Answer: B**

**Explanation**: `rclpy.spin(node)` runs the event loop that listens for incoming messages, executes callbacks, and fires timers. Without spinning, subscriber callbacks won't run and the node won't process messages. It keeps the node active until shutdown.

*Lesson Reference: 03-python-rclpy-bridge.md (Key Principles - Spinning Nodes)*
</details>

---

**Question 10 (Multiple Choice)** - 1 point

When creating a subscriber in rclpy, what is the purpose of the callback function?

A) To initialize the node
B) To execute automatically when a message arrives on the subscribed topic
C) To publish messages to other nodes
D) To create timers for periodic tasks

<details>
<summary>Reveal Answer</summary>

**Correct Answer: B**

**Explanation**: Callback functions are registered with `create_subscription()` and execute automatically (asynchronously) when messages arrive. This asynchronous pattern is fundamental to ROS2—subscribers don't block waiting; callbacks run when data is available.

*Lesson Reference: 03-python-rclpy-bridge.md (Key Principles - Subscribing to Topics)*
</details>

---

**Question 11 (Code Reading)** - 2 points

Read the following rclpy code snippet and answer:

```python
from rclpy.node import Node
from std_msgs.msg import Float32

class BatteryMonitor(Node):
    def __init__(self) -> None:
        super().__init__('battery_monitor')

        self.battery_pub = self.create_publisher(
            Float32,
            '/robot/battery',
            10
        )

        self.timer = self.create_timer(2.0, self.publish_battery)
        self.battery_level = 100.0

    def publish_battery(self) -> None:
        msg = Float32()
        msg.data = self.battery_level
        self.battery_pub.publish(msg)
        self.battery_level -= 1.0
```

**Questions**:
1. What topic does this node publish to?
2. How often does it publish battery level (in seconds)?
3. What will happen to `battery_level` over time?

<details>
<summary>Reveal Answer & Rubric</summary>

**Correct Answers**:
1. `/robot/battery`
2. Every 2.0 seconds (2 seconds)
3. It decreases by 1.0 each time it publishes (simulating battery drain)

**Grading Rubric (2 points total)**:
- **2 points**: All 3 answers correct (topic name, timer period, battery behavior)
- **1 point**: 2 out of 3 correct
- **0 points**: Fewer than 2 correct

*Lesson Reference: 03-python-rclpy-bridge.md (Practical Example - HumanoidStatusPublisher)*
</details>

---

### Lesson 4: URDF for Humanoid Robots (Questions 12-13)

**Question 12 (Multiple Choice)** - 1 point

In URDF, what is the correct unit for specifying distances (like link lengths or joint positions)?

A) Centimeters
B) Millimeters
C) Meters
D) Inches

<details>
<summary>Reveal Answer</summary>

**Correct Answer: C**

**Explanation**: URDF always uses **meters** for distances, **radians** for angles, and **kilograms** for mass. Getting units wrong is a common beginner mistake that causes robots to appear distorted or behave incorrectly in simulation.

*Lesson Reference: 04-urdf-humanoid-basics.md (Expert Insight - URDF Units and Coordinate Frame Conventions)*
</details>

---

**Question 13 (Code Reading)** - 2 points

Read the following URDF snippet:

```xml
<joint name="elbow" type="revolute">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  <origin xyz="0 0 -0.28" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="2.356" effort="30" velocity="2.0"/>
</joint>
```

**Questions**:
1. What is the parent link of this joint?
2. What type of joint is it, and what does that mean for how it moves?
3. Around which axis does this joint rotate?

<details>
<summary>Reveal Answer & Rubric</summary>

**Correct Answers**:
1. `upper_arm`
2. `revolute` - it's a hinge joint that rotates (like an elbow bending)
3. Y-axis (the `<axis xyz="0 1 0"/>` specifies rotation around Y)

**Grading Rubric (2 points total)**:
- **2 points**: All 3 answers correct
- **1 point**: 2 out of 3 correct
- **0 points**: Fewer than 2 correct

*Lesson Reference: 04-urdf-humanoid-basics.md (Key Principles - Joints, Practical Example)*
</details>

---

### Integration Questions (Questions 14-15)

**Question 14 (Short Answer)** - 2 points

You're designing a humanoid robot that needs to publish its joint positions to a visualization tool (RViz) while simultaneously receiving movement commands from a motion planner.

Describe the ROS2 architecture you would use. Your answer should mention:
- At least 2 nodes
- At least 2 topics
- Which node publishes/subscribes to which topic

<details>
<summary>Reveal Answer & Rubric</summary>

**Sample Answer**: Create a `joint_state_publisher` node that publishes current joint angles to `/joint_states` topic (for RViz to visualize). Create a `motion_controller` node that subscribes to `/motion_commands` topic (from motion planner) and commands the motors. RViz subscribes to `/joint_states` to display the robot. The motion planner publishes to `/motion_commands`.

**Grading Rubric (2 points total)**:
- **2 points**: Identifies 2+ nodes with clear responsibilities AND 2+ topics with correct pub/sub relationships
- **1 point**: Identifies nodes and topics but relationships unclear or partially incorrect
- **0 points**: Incorrect architecture or missing key components

*Integrates: Lessons 1 (ROS2 architecture), 2 (topics), 3 (rclpy nodes), 4 (joint states/URDF)*
</details>

---

**Question 15 (Multiple Choice - Scenario)** - 1 point

You're writing rclpy code for a delivery robot and need to choose a communication pattern. The robot receives a delivery request from a central scheduler and must confirm receipt before the scheduler sends another request. Which pattern should you use?

A) Topic with reliable QoS
B) Service (request/response)
C) Topic with best-effort QoS
D) Multiple topics (one for request, one for confirmation)

<details>
<summary>Reveal Answer</summary>

**Correct Answer: B**

**Explanation**: This is a classic service use case: discrete request requiring acknowledgment before proceeding. The scheduler calls a service, waits for confirmation, then continues. A topic wouldn't guarantee the request was received and acknowledged. While option D (two topics) could work, a service is the standard ROS2 pattern for request/response transactions.

*Integrates: Lessons 2 (topics vs services), 3 (rclpy implementation), 5 (delivery robot scenario from capstone)*
</details>

---

## Answer Key Summary

| Question | Correct Answer | Points | Lesson(s) |
|----------|---------------|--------|-----------|
| 1 | B | 1 | Lesson 1 |
| 2 | C | 1 | Lesson 1 |
| 3 | (Short answer - see rubric) | 2 | Lesson 1 |
| 4 | (Short answer - see rubric) | 1 | Lesson 1 |
| 5 | B | 1 | Lesson 2 |
| 6 | C | 1 | Lesson 2 |
| 7 | B | 1 | Lesson 2 |
| 8 | (Short answer - see rubric) | 1 | Lesson 2 |
| 9 | B | 1 | Lesson 3 |
| 10 | B | 1 | Lesson 3 |
| 11 | (Code reading - see rubric) | 2 | Lesson 3 |
| 12 | C | 1 | Lesson 4 |
| 13 | (Code reading - see rubric) | 2 | Lesson 4 |
| 14 | (Short answer - see rubric) | 2 | Integration |
| 15 | B | 1 | Integration |
| **Total** | | **18 points** | |

**Passing Score**: 12/18 points minimum (67% rounded to support partial credit on short answer questions)

**Note on Scoring**: While there are 15 questions, short answer and code reading questions are worth 1-2 points each, totaling 18 possible points. Passing requires 12/18 (67%), which ensures students demonstrate solid understanding even if they struggle with a few harder questions.

---

## Grading Rubric Details

### Multiple Choice Questions (Questions 1, 2, 5, 6, 7, 9, 10, 12, 15)
- **1 point**: Correct answer
- **0 points**: Incorrect answer
- **Total**: 9 questions × 1 point = 9 points

### Short Answer Questions (Questions 3, 4, 8, 14)
- **Question 3** (2 points): See detailed rubric in answer
- **Question 4** (1 point): See detailed rubric in answer
- **Question 8** (1 point): See detailed rubric in answer
- **Question 14** (2 points): See detailed rubric in answer
- **Total**: 6 points

### Code Reading Questions (Questions 11, 13)
- **Question 11** (2 points): All 3 sub-questions correct = 2pts, 2 correct = 1pt
- **Question 13** (2 points): All 3 sub-questions correct = 2pts, 2 correct = 1pt
- **Total**: 4 points

### Partial Credit Policy
- Multiple choice: No partial credit (all or nothing)
- Short answer: Partial credit awarded based on rubrics (demonstrates partial understanding)
- Code reading: Partial credit for getting 2 out of 3 correct

---

## Study Guide

If you scored below passing (12/18 points), review these areas:

**Scored low on Questions 1-4?** Review [Lesson 1: ROS2 Fundamentals](./ros2-fundamentals)
- Focus on: What ROS2 is (middleware), DDS, modularity, ROS2 graph

**Scored low on Questions 5-8?** Review [Lesson 2: Nodes, Topics, and Services](./nodes-topics-services)
- Focus on: Topics vs services, when to use each, QoS policies

**Scored low on Questions 9-11?** Review [Lesson 3: Python rclpy Bridge](./python-rclpy-bridge)
- Focus on: `rclpy.spin()`, callbacks, publishers, subscribers, timers
- Practice reading Python code examples

**Scored low on Questions 12-13?** Review [Lesson 4: URDF for Humanoid Robots](./urdf-humanoid-basics)
- Focus on: URDF units (meters, radians), links, joints, parent-child relationships
- Practice reading URDF XML

**Scored low on Questions 14-15?** Review integration concepts
- Revisit [Capstone Project](./capstone-project) for system design patterns
- Practice combining concepts from multiple lessons

---

## Next Steps

**Passed the quiz?** Congratulations! You've demonstrated solid understanding of ROS2 fundamentals. You're ready to:
- Complete the [Capstone Project](./capstone-project) if you haven't already
- Explore advanced ROS2 topics (motion planning with MoveIt, SLAM, computer vision integration)
- Build your own robot project using ROS2 Humble

**Need to retake?** Review the lessons indicated in your weak areas, then retake the quiz. Focus on understanding concepts, not memorizing answers. The quiz is designed to ensure you have the foundation needed for advanced robotics work.
