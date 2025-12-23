---
id: python-rclpy-bridge
title: "Python rclpy Bridge"
sidebar_position: 3
description: "Learn how to connect Python programming to ROS2 using the rclpy library, creating nodes, publishers, and subscribers to bring robot communication patterns to life in actual code."
time_estimate: "60 minutes"
difficulty_level: "beginner"
prerequisites: ["01-ros2-fundamentals", "02-nodes-topics-services", "Python 3.11+", "Object-oriented programming basics"]
related_lessons: ["01-ros2-fundamentals", "02-nodes-topics-services", "04-urdf-humanoid-robots"]
assessment_method: "quiz"
skills:
  - name: "rclpy API Usage"
    proficiency_level: "beginner"
    category: "programming"
    bloom_level: "apply"
    digcomp_area: "programming"
    measurable_at_this_level: "read and understand rclpy code, identify node initialization, publisher/subscriber creation, and explain what each line does"
  - name: "Python-ROS2 Integration"
    proficiency_level: "beginner"
    category: "system-integration"
    bloom_level: "apply"
    digcomp_area: "programming"
    measurable_at_this_level: "connect Python programming knowledge to ROS2 concepts, understand how Python code creates nodes and topics"
learning_objectives:
  - objective: "Understand what rclpy is and how it bridges Python to ROS2 middleware"
    proficiency_level: "beginner"
    bloom_level: "understand"
    assessment_method: "quiz questions 9-10"
  - objective: "Read and understand basic rclpy code for publishers and subscribers"
    proficiency_level: "beginner"
    bloom_level: "apply"
    assessment_method: "quiz question 11 (code reading)"
  - objective: "Explain the callback pattern in ROS2 subscribers and why it's asynchronous"
    proficiency_level: "beginner"
    bloom_level: "understand"
    assessment_method: "quiz + practice exercise"
  - objective: "Identify node initialization, publisher creation, and message publishing in Python code"
    proficiency_level: "beginner"
    bloom_level: "apply"
    assessment_method: "capstone project code analysis"
cognitive_load:
  new_concepts: 5
  assessment: "moderate - builds on ROS2 fundamentals (Lessons 1-2) and Python knowledge, introduces rclpy API patterns"
differentiation:
  extension_for_advanced: "Explore rclpy executors (SingleThreadedExecutor, MultiThreadedExecutor) for concurrent callback processing. Investigate parameter servers and dynamic reconfiguration in rclpy."
  remedial_for_struggling: "Review Python object-oriented programming (classes, methods, __init__) and callback functions before continuing. Practice writing simple Python classes."
tags: ["ros2", "rclpy", "python", "programming", "publisher", "subscriber", "nodes", "callbacks"]
generated_by: "manual"
created: "2025-12-06"
last_modified: "2025-12-06"
ros2_version: "humble"
---

# Python rclpy Bridge

Bringing ROS2 concepts to life in Python code—understanding how the rclpy library transforms your Python programs into ROS2 nodes that communicate through topics and services.

## What Is rclpy?

**rclpy** (ROS Client Library for Python) is the official Python API for ROS2. It's the bridge that connects your Python programming knowledge to the ROS2 middleware, allowing you to write robot software using familiar Python syntax while rclpy handles the complex DDS communication behind the scenes.

Think of rclpy as a translator. You write Python code using standard object-oriented patterns—classes, methods, callbacks—and rclpy translates these into ROS2 nodes, topics, services, and messages. When you create a Python class that inherits from `rclpy.node.Node`, you're not just writing a Python class; you're creating a ROS2 node that will participate in the distributed robot system you learned about in Lessons 1 and 2.

At its core, rclpy provides **Python classes and functions** that wrap the underlying ROS2 C++ implementation (rcl - ROS Client Library). This means you get the performance of native ROS2 with the productivity of Python. Your code remains Pythonic—readable, concise, with type hints and familiar patterns—while rclpy ensures it integrates seamlessly with the broader ROS2 ecosystem.

**The rclpy workflow** follows a consistent pattern: initialize the ROS2 context with `rclpy.init()`, create node objects that inherit from `rclpy.node.Node`, set up publishers and subscribers using node methods, and spin the node to process callbacks. This structure mirrors how professional roboticists build ROS2 systems, making the code you write in this course directly applicable to real robots.

What makes rclpy particularly valuable for students with Python backgrounds is its **tight integration with the Python ecosystem**. You can combine rclpy with NumPy for numerical processing, OpenCV for computer vision, TensorFlow or PyTorch for AI models, and any other Python library. This integration is why rclpy is the primary choice for research robotics and rapid prototyping—you leverage Python's vast ecosystem while gaining access to ROS2's communication infrastructure.

Perhaps most importantly, rclpy is **actively maintained and widely adopted**. When you encounter ROS2 tutorials, research papers, or open-source robot code, you'll frequently see rclpy examples. Learning rclpy means you can read and understand the majority of ROS2 Python code you'll encounter in the robotics community.

## Why rclpy Matters for Physical AI

Building a humanoid robot requires coordinating sensors, actuators, planning algorithms, and AI models—all written in different programming languages and running on different computers. For students and researchers with Python expertise, rclpy is the key that unlocks access to this distributed ecosystem without requiring mastery of C++ or low-level networking.

**Rapid Prototyping and Iteration**: Python's interactive nature combined with rclpy's straightforward API means you can test robot behaviors in minutes, not hours. Write a simple publisher node, run it, see data flowing through topics in real-time, modify the code, and restart. This tight feedback loop is essential for learning and experimentation. Compared to compiled languages where each change requires rebuilding binaries, rclpy lets you iterate at the speed of thought.

**AI and Machine Learning Integration**: Modern humanoid robots rely heavily on AI—computer vision for perception, reinforcement learning for locomotion, natural language processing for interaction. These AI models are predominantly built in Python using frameworks like PyTorch, TensorFlow, and scikit-learn. rclpy makes it trivial to wrap these models in ROS2 nodes: subscribe to camera topics for input, run inference, publish detected objects to downstream planning nodes. Without rclpy, integrating Python-based AI into a robot system would require custom networking code and serialization—weeks of work that rclpy reduces to a few lines.

**Leveraging Python's Scientific Ecosystem**: Robotics is fundamentally about processing data—sensor readings, coordinate transformations, trajectory planning. Python's scientific stack (NumPy, SciPy, Matplotlib) excels at these tasks. With rclpy, you receive sensor data as ROS2 messages, convert them to NumPy arrays with a single line, perform signal processing or filtering, and publish the results back to ROS2 topics. This seamless data flow between ROS2 and Python's scientific libraries is what makes rclpy indispensable for research and development.

**Educational Value and Community**: For students learning robotics in 2025, Python is often their first (and sometimes only) programming language. rclpy removes the barrier of learning C++ before you can experiment with robots. You can focus on **robotics concepts**—nodes, topics, services, coordinate frames—without simultaneously wrestling with pointers, memory management, and compilation toolchains. As your skills grow, you can transition to rclcpp (C++) for performance-critical components while keeping your Python code for higher-level logic.

**Production Viability**: While Python is often dismissed as "just for prototyping," modern robotics systems routinely use rclpy in production. High-level planners, web interfaces, data logging, and AI inference nodes run in Python, while low-level motor control and real-time safety systems use C++. rclpy's performance is sufficient for components that don't require microsecond latency, and its maintainability advantages often outweigh the performance cost. Learning rclpy isn't just for learning—it's a skill you'll use in professional robot development.

## Key Principles

### Initializing the ROS2 Context

Every rclpy program begins with **`rclpy.init()`**, which initializes the ROS2 middleware and prepares your Python process to communicate with other nodes. Think of this as "plugging into" the ROS2 network—without it, your code is just Python; with it, your code becomes a participant in the robot's nervous system.

```python
import rclpy

# Initialize ROS2 for this process
rclpy.init(args=None)

# ... create nodes, spin ...

# Shutdown ROS2 when done
rclpy.shutdown()
```

You call `rclpy.init()` once at program startup, create your nodes, run them, and call `rclpy.shutdown()` when exiting. This pattern ensures clean resource management—DDS connections are established on init and torn down on shutdown.

### Creating Nodes with the Node Class

In rclpy, every node is a Python class that inherits from **`rclpy.node.Node`**. This class provides methods for creating publishers, subscribers, services, and timers. By subclassing Node, you get access to the entire ROS2 API while organizing your robot's functionality as modular, reusable components.

```python
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        # Call parent constructor with node name
        super().__init__('my_robot_node')

        # Node is now registered with ROS2
        self.get_logger().info('Robot node started!')
```

The node name (`'my_robot_node'`) identifies this node in the ROS2 graph. Tools like `ros2 node list` will show it, and other nodes can discover it automatically via DDS.

### Publishing Messages with create_publisher()

To broadcast data on a topic, use **`create_publisher()`** which returns a publisher object. You specify the message type (like `String`, `Image`, `Twist`) and the topic name. The publisher doesn't know or care who (if anyone) is listening—it just broadcasts.

```python
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')

        # Create publisher: String messages on '/robot_status' topic
        self.publisher = self.create_publisher(
            String,           # Message type
            '/robot_status',  # Topic name
            10                # Queue size (history depth)
        )

        # Create timer to publish every 1.0 seconds
        self.timer = self.create_timer(1.0, self.publish_status)

    def publish_status(self):
        msg = String()
        msg.data = 'Robot operational'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
```

Notice the **timer callback pattern**: `create_timer(1.0, self.publish_status)` calls `publish_status()` every 1.0 seconds. This is how nodes perform periodic tasks—timers trigger callbacks asynchronously, keeping your node responsive while publishing data at regular intervals.

### Subscribing to Topics with create_subscription()

To receive data from a topic, use **`create_subscription()`** which registers a callback function that runs whenever a message arrives. This asynchronous pattern is fundamental to ROS2—your subscriber doesn't block waiting for messages; instead, rclpy calls your callback when data is available.

```python
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        # Create subscription: String messages from '/robot_status'
        self.subscription = self.create_subscription(
            String,                # Message type
            '/robot_status',       # Topic name
            self.status_callback,  # Callback function
            10                     # Queue size
        )

    def status_callback(self, msg: String):
        # Called automatically when messages arrive
        self.get_logger().info(f'Received: {msg.data}')
```

The **callback signature** `status_callback(self, msg: String)` receives the message as a parameter. Type hints (`msg: String`) aren't required but are best practice—they make your code more readable and enable IDE autocompletion.

### Spinning Nodes to Process Callbacks

After creating nodes, you must **spin** them to process incoming messages and timer callbacks. `rclpy.spin()` runs an event loop that listens for messages, executes callbacks, and keeps your node alive until shutdown.

```python
import rclpy
from my_package.my_node import SubscriberNode

def main():
    rclpy.init()

    node = SubscriberNode()

    # Process callbacks indefinitely
    rclpy.spin(node)

    # Cleanup (reached on Ctrl+C or shutdown)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Without spinning, your subscribers won't receive messages and timers won't fire. Think of spin as the "heartbeat" that keeps your node alive and responsive.

### 💬 AI Colearning Prompt

> **Suggested Exploration**: Ask Claude or ChatGPT to explain the callback pattern in ROS2 subscribers and why it's asynchronous. Specifically: "In the subscriber example above, what happens if the callback function takes a long time to execute? Does it block other messages from being received? How does this relate to the publish-subscribe pattern we learned in Lesson 2?"
>
> Then extend: "What would happen if I used a while loop to constantly check for messages instead of using callbacks? Why is the callback pattern better for robot systems that handle multiple sensors?"
>
> This exploration will deepen your understanding of asynchronous programming in robotics.

### 🎓 Expert Insight

**Type Hints and Python 3.11+ Best Practices in rclpy**

Modern rclpy code should leverage Python 3.11+ features for clarity, safety, and maintainability. While rclpy works without type hints, adding them transforms your code from "it runs" to "it's professional."

**Type Hints for Message Types**: Always annotate message parameters in callbacks:

```python
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def status_callback(self, msg: String) -> None:
    # IDE knows msg.data is a string
    self.get_logger().info(f'Status: {msg.data}')

def velocity_callback(self, msg: Twist) -> None:
    # IDE knows msg.linear.x is a float
    linear_speed = msg.linear.x
```

This isn't just documentation—modern IDEs use these hints for autocompletion and error detection. When you type `msg.`, your IDE shows you all available fields (`msg.data`, `msg.linear.x`, etc.) specific to that message type.

**Node Lifecycle Type Hints**: Annotate node creation for clarity:

```python
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String

class TypedNode(Node):
    def __init__(self) -> None:
        super().__init__('typed_node')

        self.publisher: Publisher = self.create_publisher(
            String,
            '/topic',
            10
        )
```

**For students in 2025**: Start with type hints from day one. They catch errors before runtime (like publishing wrong message types), make code self-documenting, and prepare you for professional codebases where type checking with tools like `mypy` is standard practice.

## Practical Example

Let's build a realistic example: a **humanoid robot status publisher** that broadcasts operational state and battery level. This demonstrates combining publishers, timers, and message construction in a single node.

**The Scenario**: Your humanoid robot needs to continuously broadcast its operational status and battery percentage so monitoring systems, dashboards, and safety nodes can track its health. This is a common pattern in production robots—status topics let you monitor robot state without polling or complex queries.

**The Implementation**:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import random  # Simulating battery sensor

class HumanoidStatusPublisher(Node):
    """
    Publishes humanoid robot operational status and battery level.

    Topics:
      - /robot/status (String): "operational" or "error"
      - /robot/battery (Float32): Battery percentage 0.0-100.0
    """

    def __init__(self) -> None:
        super().__init__('humanoid_status_publisher')

        # Create publishers for status and battery
        self.status_pub = self.create_publisher(
            String,
            '/robot/status',
            10  # QoS history depth
        )

        self.battery_pub = self.create_publisher(
            Float32,
            '/robot/battery',
            10
        )

        # Publish status at 1 Hz, battery at 0.5 Hz
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.battery_timer = self.create_timer(2.0, self.publish_battery)

        self.battery_level = 100.0  # Start fully charged

        self.get_logger().info('Humanoid status publisher started')

    def publish_status(self) -> None:
        """Publish operational status."""
        msg = String()
        msg.data = 'operational'  # In real system, check actual health

        self.status_pub.publish(msg)
        self.get_logger().info(f'Status: {msg.data}')

    def publish_battery(self) -> None:
        """Publish battery level (simulated drain)."""
        msg = Float32()

        # Simulate battery drain (in real system, read from sensor)
        self.battery_level -= random.uniform(0.1, 0.5)
        self.battery_level = max(0.0, self.battery_level)

        msg.data = self.battery_level

        self.battery_pub.publish(msg)
        self.get_logger().info(f'Battery: {msg.data:.1f}%')


def main(args=None):
    rclpy.init(args=args)

    node = HumanoidStatusPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Why This Design Works**: Notice how the node uses **two separate timers** with different frequencies—status updates every 1 second (1 Hz), battery every 2 seconds (0.5 Hz). This mirrors real robots where different data streams have different natural update rates. You don't need complex threading; timers run callbacks asynchronously in the same process.

The **type hints** (`-> None`, `msg: String`) make the code self-documenting. Any developer reading this immediately knows `publish_status()` returns nothing and `msg` is a String message.

The **docstring** explains what the node does and which topics it publishes to. This is critical in robot systems where dozens of nodes run simultaneously—documentation helps others understand how to integrate with your node.

### 🤝 Practice Exercise

**Extend the Status Publisher**

Modify the `HumanoidStatusPublisher` example to add a new topic `/robot/joint_positions` that publishes the current angles of the robot's joints.

**Your Task**:
1. Import the `Float32MultiArray` message type from `std_msgs.msg`
2. Create a new publisher for `/robot/joint_positions`
3. Add a timer (0.5 Hz) that publishes simulated joint angles
4. The message should contain an array of 12 float values (representing 12 joints)

**Consider**:
- Should joint positions update faster or slower than battery level? Why?
- How would a subscriber node use this joint position data?
- What happens if you publish joint positions at 100 Hz instead of 0.5 Hz?

Write your modified code, then ask an AI assistant to review your implementation. This exercise reinforces publisher creation, timer management, and message construction—core rclpy patterns.

## Summary

**Key Takeaways**:

- **rclpy bridges Python to ROS2**: It's the official Python API that translates Pythonic code into ROS2 nodes, topics, and services, leveraging the DDS middleware while keeping your code readable and maintainable.

- **Node class provides the foundation**: Every rclpy node inherits from `rclpy.node.Node`, gaining access to methods for creating publishers, subscribers, timers, and services. This object-oriented pattern organizes robot functionality as modular components.

- **Publishers broadcast, subscribers receive**: `create_publisher()` sends messages to topics without knowing who's listening; `create_subscription()` registers callbacks that execute asynchronously when messages arrive. This decoupling enables flexible, scalable architectures.

- **Timers drive periodic behavior**: `create_timer()` schedules callbacks at fixed intervals, perfect for publishing sensor data, updating state, or performing periodic checks. Multiple timers can coexist with different frequencies in a single node.

- **Spinning processes callbacks**: `rclpy.spin()` runs the event loop that receives messages, triggers callbacks, and fires timers. Without spinning, your node is deaf and inactive—spin is what brings it to life.

**What You Should Now Understand**: You can read and understand basic rclpy code, identifying node initialization, publisher/subscriber creation, callback patterns, and message publishing. You understand how Python classes become ROS2 nodes and how the callback pattern enables asynchronous communication. You're ready to see these patterns applied to robot modeling in the next lesson.

## Next Steps

You've learned **how to write** ROS2 nodes in Python using rclpy. In the next lesson, "URDF for Humanoid Robots," we'll shift from code to **robot modeling**. You'll learn how robots are described structurally using URDF (Unified Robot Description Format) to define joints, links, and kinematic chains—essential for simulation, visualization, and motion planning.

This progression from communication (Lessons 1-2) to programming (Lesson 3) to modeling (Lesson 4) mirrors professional robotics workflows: understand the architecture, implement the software, then define the physical structure. With rclpy mastered, you'll bring your code to life by applying it to actual robot models in the final lesson.
