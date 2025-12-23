# Nodes, Topics, and Services - Summary

**Quick Reference**: Key concepts from this lesson

## Core Concept

ROS2 nodes are independent processes that communicate through topics (publish/subscribe for continuous data) and services (request/response for commands), enabling modular robot architectures where components coordinate without tight coupling.

## Key Points

- **Nodes as Independent Processes**: Each node runs separately with isolated memory and single responsibility, providing fault isolation—if one node crashes, others continue operating.

- **Topics for Continuous Data (Pub/Sub)**: Publishers broadcast messages asynchronously to named topics; multiple subscribers receive data without knowing the source. Perfect for sensor streams (cameras, IMU, encoders).

- **Services for Commands (Request/Response)**: Synchronous request-response pattern where client waits for confirmation. Use for discrete actions requiring acknowledgment (queries, calibrations, movement commands).

- **Quality of Service (QoS) Policies**: Tune reliability vs. performance tradeoffs per-topic—reliable delivery for critical commands, best-effort for high-frequency sensor data tolerating loss.

- **Pattern Selection Impacts System Behavior**: Using topics for commands causes lost messages; using services for continuous data blocks unnecessarily. Choose based on: continuous vs. discrete, acknowledgment needs, timing requirements.

## When to Use

**Topics (Pub/Sub)**:
- Continuous sensor data streams (camera, LiDAR, IMU, encoders)
- State updates published at regular intervals
- One-to-many or many-to-many communication
- Data where latest value matters more than guaranteed delivery

**Services (Request/Response)**:
- Discrete commands requiring confirmation (move to pose, grasp object)
- Queries for current state (get position, check battery level)
- Operations completing quickly (< 1 second)
- One-to-one transactions needing acknowledgment

## Common Patterns

- **Sensor nodes publish continuously** to topics at fixed rates without blocking
- **Processing nodes subscribe** to multiple topics, fuse data, publish results
- **Command nodes use services** to request actions and wait for completion
- **Status topics use reliable QoS** with transient-local durability for late-joiners
- **High-frequency topics use best-effort QoS** to tolerate loss for lower latency

## Related Concepts

- **Previous Lesson**: ROS2 Fundamentals - what ROS2 is and why modularity matters
- **Next Lesson**: Python rclpy Bridge - implementing these patterns in actual Python code
- **Builds To**: URDF robot modeling (Lesson 4), Capstone project with full system integration
