# Python rclpy Bridge - Summary

**Quick Reference**: Key concepts from this lesson

## Core Concept

rclpy (ROS Client Library for Python) is the official Python API for ROS2 that bridges Python programming to ROS2 middleware, allowing you to create nodes, publishers, and subscribers using familiar Python syntax while rclpy handles DDS communication automatically.

## Key Points

- **rclpy.init() and Node Class**: Every rclpy program starts with `rclpy.init()` to initialize ROS2, then creates nodes by inheriting from `rclpy.node.Node`, gaining access to the entire ROS2 API.

- **Publishers Broadcast Data**: `create_publisher(MessageType, '/topic', queue_size)` creates a publisher that broadcasts messages asynchronously to topics without knowing who's listening.

- **Subscribers Use Callbacks**: `create_subscription(MessageType, '/topic', callback_function, queue_size)` registers callbacks that execute asynchronously when messages arrive, enabling non-blocking message processing.

- **Timers Drive Periodic Actions**: `create_timer(period_sec, callback_function)` schedules callbacks at fixed intervals—perfect for publishing sensor data at regular rates (1 Hz, 10 Hz, 100 Hz).

- **Spinning Processes Events**: `rclpy.spin(node)` runs the event loop that receives messages, triggers callbacks, and fires timers. Without spinning, nodes are inactive.

## When to Use

**Publishers**:
- Broadcasting sensor data (camera frames, IMU readings, joint positions)
- Publishing robot state updates (battery level, operational status)
- Sending continuous data streams that any node can subscribe to

**Subscribers**:
- Receiving sensor data for processing (vision algorithms, sensor fusion)
- Monitoring robot state (dashboards, safety systems)
- Consuming data streams to make decisions (planning, control)

**Timers**:
- Publishing at fixed rates (1 Hz status updates, 30 Hz camera frames)
- Periodic health checks and state updates
- Scheduling recurring tasks without blocking

## Common Patterns

- **Node inherits from `rclpy.node.Node`** and sets up publishers/subscribers in `__init__()`
- **Publishers created once** in `__init__`, messages published repeatedly in timer callbacks
- **Subscribers created once** with callback functions that process incoming messages
- **Type hints used for clarity** (`msg: String`, `-> None`) to document message types
- **Logging with `self.get_logger().info()`** for debugging and monitoring
- **main() function handles lifecycle**: `init()` → create node → `spin()` → `shutdown()`

## Related Concepts

- **Previous Lessons**: ROS2 Fundamentals (Lesson 1), Nodes/Topics/Services (Lesson 2)—rclpy brings these concepts to life in Python code
- **Next Lesson**: URDF for Humanoid Robots (Lesson 4)—applying rclpy code to robot models
- **Builds To**: Capstone project where students write multi-node Python systems
