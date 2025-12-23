---
id: nodes-topics-services
title: "Nodes, Topics, and Services"
sidebar_position: 2
description: "Learn how ROS2 nodes communicate through topics (pub/sub) and services (request/response), and understand when to use each communication pattern for coordinating robot behaviors."
time_estimate: "45 minutes"
difficulty_level: "beginner"
prerequisites: ["01-ros2-fundamentals", "Python basics", "Understanding of pub/sub model"]
related_lessons: ["01-ros2-fundamentals", "03-python-rclpy-bridge"]
assessment_method: "quiz"
skills:
  - name: "Node Architecture Design"
    proficiency_level: "beginner"
    category: "system-design"
    bloom_level: "understand"
    digcomp_area: "technical-concepts"
    measurable_at_this_level: "explain what nodes are, their lifecycle, and how they coordinate in a robot system"
  - name: "Communication Pattern Selection"
    proficiency_level: "beginner"
    category: "system-design"
    bloom_level: "apply"
    digcomp_area: "problem-solving"
    measurable_at_this_level: "choose appropriate communication pattern (topic vs service) for different robot scenarios"
learning_objectives:
  - objective: "Understand what nodes are and how they enable modular robot architectures"
    proficiency_level: "beginner"
    bloom_level: "understand"
    assessment_method: "quiz questions 5-6"
  - objective: "Explain the difference between topics (pub/sub) and services (request/response) communication patterns"
    proficiency_level: "beginner"
    bloom_level: "understand"
    assessment_method: "quiz questions 7-8"
  - objective: "Apply pattern selection knowledge to choose topics vs services for specific robot scenarios"
    proficiency_level: "beginner"
    bloom_level: "apply"
    assessment_method: "practice exercise + capstone project"
  - objective: "Understand Quality of Service (QoS) policies and their role in reliable communication"
    proficiency_level: "beginner"
    bloom_level: "understand"
    assessment_method: "quiz"
cognitive_load:
  new_concepts: 4
  assessment: "moderate - builds directly on Lesson 1 ROS2 fundamentals, introduces communication pattern distinctions"
differentiation:
  extension_for_advanced: "Research advanced QoS policies (deadline, lifespan, liveliness). Explore ROS2 actions for long-running tasks. Investigate parameter services for runtime configuration."
  remedial_for_struggling: "Review Lesson 1 (ROS2 fundamentals) focusing on nodes and topics concepts before continuing."
tags: ["ros2", "nodes", "topics", "services", "communication", "pub-sub", "request-response", "qos"]
generated_by: "manual"
created: "2025-12-06"
last_modified: "2025-12-06"
ros2_version: "humble"
---

# Nodes, Topics, and Services

Mastering the communication patterns that enable robot components to coordinate effectively - understanding when to broadcast data (topics) vs. when to request specific actions (services).

## What Are Nodes?

In ROS2, a **node** is an independent process responsible for a single, well-defined task within your robot system. Think of nodes as specialized workers in a factory: one worker (node) operates the camera, another processes vision data, another controls motors, and another plans paths. Each focuses on their expertise without needing to understand what others do internally.

Every node runs in its own process with isolated memory, which provides crucial fault isolation. If your vision processing node crashes due to a corrupted image, your balance control node keeps running, preventing your humanoid robot from falling. This independence is fundamental to building reliable robotic systems where individual component failures don't cascade into total system failure.

Nodes communicate exclusively through well-defined interfaces—they cannot directly access each other's memory or call each other's functions. This enforced separation might seem restrictive if you're used to writing monolithic programs, but it's what enables the modularity that makes complex robots possible. You can develop, test, and debug each node independently, then compose them into increasingly sophisticated systems.

The **node lifecycle** in ROS2 is straightforward: nodes initialize (set up their publishers, subscribers, and internal state), run their main loop (processing messages, publishing data, responding to requests), and shutdown cleanly when asked. This predictable lifecycle makes it easy to start and stop nodes dynamically—crucial when you need to upgrade a single component without restarting your entire robot.

Nodes discover each other automatically through DDS, forming the **ROS2 graph**. When you launch a camera node and a vision processing node on the same network, they find each other without manual configuration. This automatic discovery extends across multiple computers, so your robot's onboard Raspberry Pi running motor controllers can seamlessly communicate with a powerful desktop computer running deep learning inference.

What makes nodes powerful isn't just their independence—it's how they **compose**. A simple walking controller might be one node. Add a balance compensation node that subscribes to IMU data, and your robot handles uneven terrain. Add obstacle detection and path planning nodes, and now it navigates autonomously. Each capability is an independent module that integrates through standardized communication.

## Why Node Communication Matters

A humanoid robot must coordinate dozens of subsystems simultaneously: vision sensors feed object detection, which informs navigation planning, which commands leg actuators, while balance control monitors orientation and makes real-time adjustments. No single computer can efficiently handle all this computation, and even if it could, tightly coupling everything into one program would be an unmaintainable nightmare.

**The Coordination Challenge**: Imagine trying to coordinate a humanoid's walk cycle. The gait planner generates desired foot positions 10 times per second. The balance controller needs IMU data at 100Hz to maintain stability. Camera frames arrive at 30Hz. Motor controllers expect commands at 1000Hz for smooth motion. These different update rates must coexist without one blocking another. ROS2's asynchronous communication patterns solve this through topics—each node publishes data at its natural rate, and subscribers consume it at theirs.

**Data Flow vs. Commands**: Not all robot communication is the same. A camera continuously broadcasts image data whether anyone is listening or not (data flow). But when you want your robot to grasp an object, you send a specific request and wait for confirmation that the grasp succeeded (command). Topics handle data flow; services handle commands. Choosing the wrong pattern leads to systems that work in demos but fail unpredictably in real environments.

**Reliability Requirements Vary**: When your humanoid publishes its battery level, losing an occasional message is fine—the next update arrives in a second. But when the safety system sends an emergency stop command, that message must arrive, guaranteed, exactly once. ROS2's Quality of Service (QoS) policies let you tune this tradeoff per-topic: safety-critical commands get reliable delivery, high-frequency sensor data tolerates loss for lower latency.

**Distributed Intelligence**: Modern humanoid robots often use multiple computers: embedded microcontrollers for motor control (real-time requirements), edge GPUs for vision processing (computation requirements), and cloud connectivity for learning and telemetry. ROS2's communication patterns abstract away the physical network topology—whether a subscriber is on the same computer or across WiFi, the programming model remains identical. This abstraction accelerates development and makes systems portable.

**Ecosystem Integration**: When you use ROS2's standard communication patterns, your robot automatically works with the entire ROS2 ecosystem. Visualization tools subscribe to your topics to display robot state. Data recording tools capture message streams for later analysis. Simulation systems can replace real sensors by publishing synthetic data to the same topics. This ecosystem effect multiplies as more developers follow the same patterns.

## Key Principles

### Topics: Publish-Subscribe (Many-to-Many)

**Topics are for continuous data streams**: A topic is a named bus where nodes publish data and others subscribe to receive it. Publishers broadcast messages without knowing who (if anyone) is listening. Subscribers receive messages without knowing who published them. This **loose coupling** is essential for flexibility—you can add new subscribers (like a data logger or visualizer) without modifying publishers.

**Asynchronous and non-blocking**: When a node publishes to a topic, it doesn't wait for subscribers to process the data. The publisher continues immediately, making topics perfect for high-frequency sensors (cameras, LiDAR, IMU) that can't afford to block waiting for slow consumers. Subscribers process messages in callbacks that run independently of the publishing node's timing.

**Many-to-many communication**: Multiple nodes can publish to the same topic (sensor fusion from multiple cameras), and multiple nodes can subscribe (vision processing, obstacle detection, and recording all listening to camera data). This flexibility enables modular architectures where capabilities can be added or removed without restructuring the entire system.

### Services: Request-Response (One-to-One)

**Services are for explicit requests**: Unlike topics where data flows continuously, a service represents an action a node can perform on request. Think of services like function calls across nodes: a client sends a request with parameters, the service processes it, and sends back a response. Common uses include requesting a robot's current state, triggering a calibration routine, or commanding a discrete action.

**Synchronous and blocking**: When a node calls a service, it waits (blocks) until the service responds or times out. This synchronous behavior is appropriate for operations that complete quickly and where the requester needs confirmation before proceeding. However, it's dangerous for long-running operations—blocking for minutes while waiting for a "navigate to waypoint" service would freeze your node.

**One-to-one communication**: Each service request goes to exactly one service server, and that server sends exactly one response back to the requester. While multiple nodes can call the same service, each call is an independent transaction. This pattern ensures predictable behavior for commands that require acknowledgment.

### Quality of Service (QoS) Policies

**Tuning reliability vs. performance**: Every topic and service can specify QoS policies that control how messages are delivered. **Reliable** delivery guarantees messages arrive (like TCP), retransmitting if needed—essential for commands. **Best effort** delivery skips retransmission (like UDP), accepting occasional loss for lower latency—appropriate for high-frequency sensor data where the next reading is more valuable than a delayed old one.

**History depth and durability**: QoS policies also control how many messages are buffered (**history depth**) and whether late-joining subscribers receive previous messages (**durability**). A robot state publisher might use "transient local" durability so new subscribers immediately receive the current state without waiting for the next publish cycle.

### 💬 AI Colearning Prompt

> **Suggested Exploration**: Ask Claude or ChatGPT to explain when you should use topics vs. services for a robot arm control scenario. Specifically: "I have a robot arm that needs to (1) report its current joint angles, (2) respond to commands to move to specific positions, and (3) send alerts when it detects excessive force. Should each of these use topics or services? Why?"
>
> Then extend the question: "What happens if I use the wrong pattern—what problems would occur if joint angles were a service instead of a topic, or if position commands were a topic instead of a service?"
>
> This exploration will solidify your understanding of communication pattern selection.

### 🎓 Expert Insight

**Quality of Service: The Hidden Complexity That Matters**

QoS policies are where ROS2's DDS foundation shows its power—and where beginners often stumble. Unlike ROS1 where communication was simple but inflexible, ROS2 lets you tune reliability, latency, and resource usage per-topic. This flexibility is crucial for production robots but adds complexity.

**The Reliability Tradeoff**: Reliable QoS guarantees message delivery through acknowledgments and retransmission—perfect for commands where "emergency stop" must never be lost. But reliable delivery adds latency and network overhead. For a camera publishing 30 frames/second, missing one frame is better than delaying all subsequent frames waiting for a retransmit. Best-effort QoS accepts loss for lower latency—use it for high-frequency sensor data.

**History Depth Matters**: A depth-1 history keeps only the latest message in the queue. For a robot pose topic, only the current position matters—old positions are irrelevant. But for a command topic where a burst of commands might arrive faster than they're processed, you need sufficient depth to buffer them. Too shallow and commands are dropped; too deep and you waste memory and might process stale commands.

**Compatibility Required**: Here's the catch—publishers and subscribers must have compatible QoS policies or they won't connect. A reliable publisher won't match a best-effort subscriber (reliability mismatch). A depth-10 publisher won't match a depth-1 subscriber with incompatible durability. These "incompatible QoS" errors are a common source of frustration when topics mysteriously don't connect despite correct names.

**For beginners in 2025**: Start with default QoS policies (reliable, volatile, depth-10) and only tune when you hit specific problems—cameras dropping frames due to network overhead, commands being lost, or excessive memory usage. The ROS2 Humble defaults are sensible for most use cases. As you gain experience, measure first, optimize second.

## Practical Example

Let's design the communication architecture for a humanoid robot performing a manipulation task: picking up a bottle from a table.

**The Scenario**: Your humanoid has stereo cameras for vision, force sensors in its hands, arm joint controllers, and a central task planner. It must locate the bottle, reach for it, grasp with appropriate force, and confirm success.

**The Communication Architecture**:

**Topics (Continuous Data Streams)**:
- `/camera/left/image` and `/camera/right/image` (sensor_msgs/Image, 30Hz): Camera nodes publish continuously. A vision processing node subscribes to detect the bottle's 3D position. Uses **best-effort** QoS since missing a frame is acceptable—the next one arrives in 33ms.

- `/hand/force_sensor` (geometry_msgs/WrenchStamped, 100Hz): Force sensors publish measured forces continuously. The grasp controller subscribes to detect when grip force is sufficient. Best-effort QoS, depth-1 history—only current force matters.

- `/arm/joint_states` (sensor_msgs/JointState, 50Hz): Arm controllers publish current joint angles and velocities. Multiple subscribers (visualization, planning, telemetry) receive this data. Best-effort QoS—if visualization lags, it's not safety-critical.

- `/task/status` (std_msgs/String, 1Hz): Task planner publishes current task phase ("locating bottle", "reaching", "grasping"). Used for monitoring and debugging. Reliable QoS with transient-local durability so late-joining monitors see current status immediately.

**Services (Explicit Requests)**:
- `/vision/locate_object` (Request: object_name, Response: pose): When the task planner is ready to grasp, it calls this service asking vision processing to locate the bottle. The service responds with 3D coordinates or an error if the bottle isn't visible. Blocks until response received (synchronous).

- `/arm/move_to_pose` (Request: target_pose, Response: success): Task planner requests arm movement to the grasp position. The service returns success when the arm reaches the target or fails on collision/timeout. This is synchronous because the planner must wait for the arm to reach before attempting the grasp.

- `/gripper/set_force` (Request: force_newtons, Response: acknowledged): Commands the gripper to apply specific grasping force. Returns immediately after the command is accepted (doesn't wait for force to stabilize—the continuous force sensor topic monitors that).

**Why This Design Works**: Notice how continuous sensor data flows through topics at natural rates without blocking anything. Commands that require confirmation (locate, move, grasp) use services so the task planner knows when each step completes before proceeding. If vision processing crashes mid-reach, force sensors and joint states keep publishing—the arm continues its motion safely even though planning halted. When vision restarts, it reconnects automatically via DDS discovery.

### 🤝 Practice Exercise

**Design Communication Patterns for a Delivery Robot**

You're building an autonomous delivery robot for indoor environments. It has:
- LIDAR scanner for obstacle detection (360° scan, 10Hz)
- IMU for orientation tracking (100Hz)
- Wheel encoders for position estimation (50Hz)
- Navigation planner that computes paths
- Motor controllers for left/right wheels
- Delivery status indicator (LED display)
- User interface where staff request deliveries

**Your Task**: For each of the following, decide if it should be a **topic** or a **service**, and justify your choice:

1. LIDAR scan data being sent to the obstacle detection system
2. Staff requesting "deliver to Room 302"
3. Navigation planner commanding wheel velocities
4. Robot reporting its current battery level
5. Emergency stop command from a safety monitor
6. Request to the navigation system: "What's your current estimated position?"

For each, consider:
- Is it continuous data or a one-time request?
- Does the sender need acknowledgment/response?
- What happens if a message is lost?
- How often does this communication occur?

Write out your answers, then ask an AI assistant to review your reasoning. Understanding these tradeoffs is crucial for designing robust robot systems.

## Summary

**Key Takeaways**:

- **Nodes are independent processes**: Each node handles one responsibility with isolated memory and predictable lifecycle. This independence enables modular development, fault isolation, and dynamic composition of robot capabilities.

- **Topics for data streams, services for commands**: Topics use pub/sub for continuous, asynchronous data flow (sensors, state updates). Services use request/response for explicit commands that require acknowledgment (actions, queries, discrete operations).

- **Communication patterns have tradeoffs**: Topics are asynchronous (non-blocking, many-to-many, lossy unless QoS configured otherwise). Services are synchronous (blocking, one-to-one, reliable). Choose based on your use case—continuous vs. discrete, timing requirements, reliability needs.

- **Quality of Service policies tune behavior**: QoS settings control reliability (guaranteed vs. best-effort), history (buffer depth), and durability (late-joiner behavior). Default settings work for most cases; tune when you hit specific problems with latency, reliability, or resource usage.

- **Pattern selection impacts system behavior**: Using the wrong pattern causes subtle failures—services that block too long, topics that lose critical commands, mismatched QoS preventing connections. Design communication architecture early and validate it matches your operational requirements.

**What You Should Now Understand**: You can now explain how ROS2 nodes communicate through topics and services, choose the appropriate pattern for different robot scenarios, and understand why communication architecture design is foundational to reliable robotic systems. You're ready to see these concepts implemented in actual Python code in the next lesson.

## Next Steps

You've learned **how** robot components communicate through topics and services. In the next lesson, "Python rclpy Bridge," we'll translate these concepts into **working code**. You'll see how to create nodes in Python, set up publishers and subscribers, define service clients and servers, and bring these communication patterns to life in the `rclpy` library.

This progression from conceptual understanding to hands-on implementation mirrors professional robotics development: first master the architecture and patterns, then implement them with confidence knowing why each design choice matters. With communication patterns solid, you'll write robot code that's maintainable, testable, and composable.
