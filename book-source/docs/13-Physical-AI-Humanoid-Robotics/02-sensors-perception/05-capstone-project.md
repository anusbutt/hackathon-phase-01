---
id: capstone-project
title: "Capstone Project: Design a Multi-Sensor Humanoid Perception System"
sidebar_position: 5
description: "Integrative project combining camera systems, depth sensing, IMU integration, and sensor fusion to design a complete perception system for a humanoid warehouse robot."
time_estimate: "120 minutes"
difficulty_level: "intermediate"
prerequisites: ["01-camera-systems", "02-depth-sensing", "03-imu-proprioception", "04-sensor-fusion"]
related_lessons: ["01-camera-systems", "02-depth-sensing", "03-imu-proprioception", "04-sensor-fusion"]
assessment_method: "capstone project"
skills:
  - name: "Multi-Sensor System Design"
    proficiency_level: "beginner"
    category: "system-design"
    bloom_level: "apply"
    digcomp_area: "problem-solving"
    measurable_at_this_level: "design perception system with camera, depth, and IMU sensors with justified selections"
  - name: "Sensor Selection and Trade-off Analysis"
    proficiency_level: "beginner"
    category: "system-design"
    bloom_level: "analyze"
    digcomp_area: "problem-solving"
    measurable_at_this_level: "evaluate sensor options and justify choices based on task requirements"
  - name: "Sensor Fusion Architecture"
    proficiency_level: "beginner"
    category: "system-design"
    bloom_level: "apply"
    digcomp_area: "problem-solving"
    measurable_at_this_level: "propose fusion strategy combining multiple sensor modalities"
  - name: "ROS2 Multi-Sensor Integration"
    proficiency_level: "beginner"
    category: "programming"
    bloom_level: "apply"
    digcomp_area: "programming"
    measurable_at_this_level: "outline ROS2 node architecture for multi-sensor perception system"
learning_objectives:
  - objective: "Integrate knowledge of camera systems, depth sensors, IMU, and sensor fusion into cohesive perception system design"
    proficiency_level: "beginner"
    bloom_level: "apply"
    assessment_method: "capstone deliverable - complete system design with sensor configuration"
  - objective: "Analyze sensor trade-offs (range, accuracy, environmental conditions) and justify selections for specific robot tasks"
    proficiency_level: "beginner"
    bloom_level: "analyze"
    assessment_method: "capstone deliverable - sensor selection justification with trade-off analysis"
  - objective: "Design sensor fusion architecture that combines visual, depth, and inertial data for robust perception"
    proficiency_level: "beginner"
    bloom_level: "apply"
    assessment_method: "capstone deliverable - fusion strategy diagram with data flow"
  - objective: "Outline ROS2 multi-sensor integration code showing message synchronization and fusion logic"
    proficiency_level: "beginner"
    bloom_level: "apply"
    assessment_method: "capstone deliverable - Python code structure with multi-sensor subscribers"
cognitive_load:
  new_concepts: 0
  assessment: "high - integrates all 4 lessons, requires synthesis of camera, depth, IMU, and fusion concepts simultaneously"
differentiation:
  extension_for_advanced: "Implement the perception system in actual ROS2 code with simulated sensors in Gazebo. Add advanced features like visual-inertial SLAM, real-time object tracking with depth, or multi-robot sensor sharing."
  remedial_for_struggling: "Complete the project in stages: first sensor selection only, then placement/configuration, then ROS2 architecture, then fusion strategy. Review individual lessons as needed before integration."
tags: ["sensors", "perception", "capstone", "project", "integration", "multi-sensor", "camera", "depth", "imu", "sensor-fusion", "humanoid"]
generated_by: "manual"
created: "2025-12-16"
last_modified: "2025-12-16"
ros2_version: "humble"
---

# Capstone Project: Design a Multi-Sensor Humanoid Perception System

Integrate everything you've learned—camera systems, depth sensing, IMU integration, and sensor fusion—into a complete perception system for a humanoid warehouse robot.

## Project Overview

You're tasked with designing the **perception system for a humanoid warehouse robot** that picks items from shelves, navigates between aisles, and delivers packages to packing stations. This robot must recognize objects, measure distances, maintain balance while carrying loads, and robustly perceive its environment despite varying lighting, cluttered scenes, and dynamic obstacles.

Your goal is to create a comprehensive sensor configuration and fusion architecture that demonstrates your understanding of multi-modal perception, not to implement production code.

**The Scenario**: A logistics company is deploying humanoid robots (similar to Tesla Optimus or Agility Digit) in a warehouse environment. The robot must:
- **Navigate autonomously** through narrow aisles with moving forklifts and people
- **Locate and identify** specific items on shelves at various heights (floor to 2 meters)
- **Reach and grasp** objects while maintaining balance on two legs
- **Carry loads** up to 10kg while walking, requiring continuous balance control
- **Operate reliably** in varying lighting (bright windows + dark corners) and handle transparent/reflective packaging

**What You'll Design**:
1. **Sensor Configuration**: Which cameras, depth sensors, and IMUs? Where are they placed on the robot?
2. **Sensor Selection Justification**: Why did you choose each sensor type? What are the trade-offs?
3. **ROS2 Integration Architecture**: How do sensor data streams flow through ROS2 nodes?
4. **Sensor Fusion Strategy**: How do you combine camera, depth, and IMU data for robust perception?

This capstone integrates all four lessons. It's conceptual—you're designing the perception architecture and justifying decisions, not implementing every algorithm. Think like a robotics perception engineer: what sensors are needed, where should they be placed, how should data be fused, and why is this architecture appropriate for the task?

## Project Requirements

Your multi-sensor perception system design must include:

### 1. Sensor Configuration (30 points)

Design a complete sensor suite for the humanoid warehouse robot. Specify **camera(s), depth sensor(s), and IMU placement**.

**Required Sensors**:
- **At least 2 cameras** (specify type: monocular, stereo, RGB-D; resolution; FOV; placement)
- **At least 1 depth sensor** (specify type: LiDAR, structured light, ToF; range; accuracy; placement)
- **At least 1 IMU** (specify location: torso, head, limbs; purpose: balance, odometry, vibration detection)

For each sensor, provide:
- **Sensor Type and Model** (e.g., "Stereo camera - ZED 2", "2D LiDAR - SICK TiM551", "6-axis IMU - Bosch BMI088")
- **Key Specifications** (resolution/FOV for cameras, range/accuracy for depth, update rate for IMU)
- **Physical Placement** (where on the robot: head-mounted, torso-mounted, wrist-mounted, etc.)
- **Primary Purpose** (what perception task: object recognition, obstacle avoidance, grasp planning, balance control, etc.)

**Example Sensor Entry**:
- **Type**: RGB-D camera (Intel RealSense D435i)
- **Specs**: 1280×720 RGB, 1280×720 depth, 87° FOV, 0.3-3m range
- **Placement**: Head-mounted, forward-facing, 1.6m height (eye level)
- **Purpose**: Object identification (RGB) + grasp distance estimation (depth) for shelf items

### 2. Sensor Selection Justification (25 points)

For your sensor configuration, explain **why you chose each sensor type** and analyze the **trade-offs**.

Answer these questions for each sensor category:

**Camera Selection**:
- Why monocular, stereo, or RGB-D? What does this choice optimize for (cost, processing, accuracy)?
- How does camera placement (head vs wrists vs torso) support the warehouse task?
- How do you handle lighting challenges (bright windows, dark corners, backlit objects)?

**Depth Sensor Selection**:
- Why LiDAR, structured light, ToF, or stereo vision? What task requirements drove this choice?
- What are the range and accuracy trade-offs? (e.g., long-range navigation vs short-range manipulation)
- How does your depth sensor perform indoors with transparent/reflective packaging materials?

**IMU Selection and Placement**:
- Where is the IMU located (torso center-of-mass, head, feet)? Why this placement for a bipedal robot?
- What does the IMU measure that cameras and depth sensors cannot provide?
- How does IMU data support balance control when the robot carries variable loads?

**Example Justification**:
> **Camera Choice**: RGB-D camera on head for integrated color+depth. Trade-off: Limited range (3m) vs stereo, but simpler calibration and processing. Good for shelf items at 0.5-2m distance. Added wrist camera (monocular) for close-range grasp verification (&lt;0.3m) where RGB-D depth fails.

### 3. ROS2 Integration Architecture (25 points)

Design the **ROS2 node architecture** showing how sensor data flows through your perception system.

**Required Nodes (minimum 4-6)**:
- **Camera Processing Node**: Subscribes to camera images, performs object detection/recognition
- **Depth Processing Node**: Subscribes to depth data (point cloud or depth image), extracts obstacle information
- **IMU Monitor Node**: Subscribes to IMU data, estimates orientation and detects balance issues
- **Sensor Fusion Node**: Combines camera, depth, and IMU data for unified perception
- **Localization Node**: Fuses visual and inertial data for odometry (where is the robot?)
- **Object Recognition Node**: Identifies specific items on shelves using camera + depth

For each node, specify:
- **Node Name** (descriptive, like `object_detector`, `depth_processor`, `balance_monitor`)
- **Input Topics** (what sensor_msgs messages: Image, PointCloud2, Imu, LaserScan?)
- **Output Topics** (what processed data: detected objects, obstacles, orientation, fused state?)
- **Processing** (brief description: "runs YOLOv5 on RGB images", "filters point cloud for obstacles &lt;2m")

**Example Node**:
- **Node**: `visual_inertial_odometry_node`
- **Inputs**: `/camera/image_raw` (sensor_msgs/Image), `/imu/data` (sensor_msgs/Imu)
- **Outputs**: `/odom` (nav_msgs/Odometry) - robot pose estimate
- **Processing**: Tracks visual features frame-to-frame, fuses with IMU using EKF for drift-free odometry

**Communication Design**:
For each data flow between nodes, specify:
- **Topic Name** (e.g., `/camera/rgb/image_raw`, `/depth/points`, `/imu/data`)
- **Message Type** (sensor_msgs/Image, sensor_msgs/PointCloud2, sensor_msgs/Imu, custom messages)
- **Frequency** (30 Hz for cameras, 100 Hz for IMU, 10 Hz for fused state)
- **Why this pattern?** (continuous stream = topic; request-response = service; synchronization needed?)

### 4. Sensor Fusion Strategy (20 points)

Describe **how you combine data from multiple sensors** to achieve robust perception. Address synchronization, fusion algorithms, and failure handling.

**Required Fusion Strategies (choose at least 2)**:

**Visual-Inertial Fusion (Camera + IMU)**:
- How do you fuse camera motion estimates with IMU acceleration/gyroscope data?
- What fusion algorithm (complementary filter, Kalman filter, robot_localization)?
- Why is this fusion beneficial? (e.g., camera provides scale, IMU prevents drift between frames)

**Depth-Enhanced Object Detection (Camera + Depth)**:
- How do you combine RGB object detection with depth measurements?
- Do you use early fusion (RGB-D input to detector), late fusion (separate detection + depth lookup), or hybrid?
- What does depth add to pure camera detection? (e.g., reject false positives on 2D posters, measure actual grasp distance)

**Multi-Sensor Obstacle Avoidance (Depth + Camera + IMU)**:
- How do you merge LiDAR/depth camera obstacle maps with visual obstacle detection?
- How does IMU orientation inform which sensor data is most reliable (e.g., camera unreliable if robot tilted)?
- What happens when sensors disagree (camera sees clear path, LiDAR detects transparent glass)?

**Timestamp Synchronization**:
- How do you handle different sensor frequencies (cameras 30Hz, IMU 100Hz, LiDAR 10Hz)?
- Do you use message_filters for exact timestamp matching or approximate time policies?

**Sensor Failure Handling**:
- What happens if a camera is blinded by sunlight or a depth sensor fails?
- How does the system degrade gracefully (e.g., fall back to pure visual navigation if LiDAR fails)?

**Example Fusion Strategy**:
> **Visual-Inertial Odometry (VIO)**: Camera tracks visual features for position changes, IMU measures acceleration and rotation. Fusion: Extended Kalman Filter (EKF) predicts pose from IMU high-rate data, corrects with camera visual measurements at 30Hz. Benefit: IMU fills gaps between camera frames (high-rate smooth motion), camera prevents IMU drift over time (absolute position). Implementation: ROS2 `robot_localization` package with dual EKF nodes.

## Deliverables

Submit your capstone project with the following components:

1. **Sensor Configuration Diagram** (physical layout):
   - Drawing or description showing sensor placement on humanoid robot body
   - Label each sensor with type and purpose
   - Include field-of-view overlays for cameras, range indicators for depth sensors

2. **Sensor Selection Justification** (written report):
   - 2-3 paragraphs per sensor category (cameras, depth, IMU)
   - Explain trade-offs and why your choices fit the warehouse task
   - Address environmental challenges (lighting, transparent objects, dynamic obstacles)

3. **ROS2 Node Architecture Diagram**:
   - Block diagram showing nodes as boxes, topics as arrows
   - Label each topic with message type and frequency
   - Include at least 4-6 nodes with clear data flow

4. **ROS2 Code Outline** (Python pseudocode):
   - Skeleton code for 1-2 key nodes (sensor fusion node, multi-sensor subscriber)
   - Show rclpy structure: subscribers, message_filters synchronization, callback processing
   - Include type hints and comments explaining sensor integration

5. **Sensor Fusion Strategy** (written description):
   - 1-2 paragraphs describing your fusion approach
   - Specify algorithms (complementary filter, Kalman filter, weighted averaging)
   - Explain how fusion improves robustness over single sensors

## Evaluation Rubric

Your capstone project will be evaluated on:

| Category | Excellent (90-100%) | Good (80-89%) | Needs Work (&lt;80%) |
|----------|---------------------|---------------|-------------------|
| **Sensor Configuration** (30 pts) | Complete sensor suite with detailed specs, appropriate placement for all tasks, addresses environmental challenges | Sensor suite covers main tasks, reasonable placement, some environmental factors considered | Missing key sensors or unclear placement, environmental challenges not addressed |
| **Justification & Trade-offs** (25 pts) | Thorough analysis of sensor trade-offs, clear rationale for each choice tied to task requirements, addresses failure modes | Good justification for most sensors, some trade-offs analyzed, task fit explained | Weak justification, trade-offs not analyzed, unclear why sensors fit task |
| **ROS2 Architecture** (25 pts) | Clear node diagram with 5+ nodes, appropriate topics/frequencies, demonstrates understanding of ROS2 message flow | Node architecture covers main functions, 4+ nodes, topics mostly correct | Unclear architecture, missing key nodes, ROS2 message types incorrect |
| **Sensor Fusion Strategy** (20 pts) | Detailed fusion approach with algorithms specified, synchronization handled, failure modes addressed | Fusion strategy described for 1-2 sensor pairs, some algorithm detail | Vague fusion description, no specific algorithms or synchronization |

**Total: 100 points**

**Pass Threshold**: 80 points (demonstrates integration of all 4 lessons)

## Resources and References

**Review These Lessons**:
- [Lesson 1: Camera Systems](./01-camera-systems.md) - Camera types, sensor_msgs/Image, placement strategies
- [Lesson 2: Depth Sensing](./02-depth-sensing.md) - LiDAR vs depth cameras, sensor_msgs/PointCloud2, range/accuracy trade-offs
- [Lesson 3: IMU & Proprioception](./03-imu-proprioception.md) - Accelerometer/gyroscope, sensor_msgs/Imu, balance control
- [Lesson 4: Sensor Fusion](./04-sensor-fusion.md) - Kalman filters, VIO, robot_localization, message_filters

**Real-World Examples**:
- **Tesla Optimus**: Multi-camera configuration (head + wrists), vision-centric fusion for manipulation
- **Agility Robotics Digit**: Stereo cameras + IMU for VIO, LiDAR for navigation, depth for grasp planning
- **Boston Dynamics Atlas**: High-grade IMU + stereo vision for SLAM, depth sensors for obstacle avoidance

**ROS2 Packages to Reference**:
- `image_transport` - Efficient image streaming
- `depth_image_proc` - Depth image processing (point cloud conversion, obstacle extraction)
- `robot_localization` - Sensor fusion (EKF, UKF) for camera+IMU+wheel odometry
- `message_filters` - Timestamp synchronization for multi-sensor topics

**Optional Setup** (for advanced students who want to implement):
If you want to test your design in simulation:
1. Install ROS2 Humble and Gazebo
2. Use pre-built humanoid robot models (e.g., Robonaut, NASA Valkyrie URDF)
3. Add simulated sensors to URDF (camera, depth camera, IMU plugins)
4. Implement your ROS2 nodes and test with simulated sensor data
5. Visualize in RViz to see camera images, point clouds, TF frames

## Tips for Success

💬 **AI Colearning Prompt**: "Ask Claude to critique your sensor configuration for the warehouse task. Prompt: 'I'm designing a humanoid perception system with [your sensors]. What are the biggest weaknesses in this configuration for indoor warehouse navigation and manipulation? What environmental conditions might cause failures?'"

🎓 **Expert Insight**: Real perception systems layer redundancy. Tesla Optimus uses 8+ cameras with overlapping fields-of-view so losing one camera doesn't create blind spots. Consider: if your primary depth sensor fails or is occluded, can the robot still navigate safely? If the head camera is blinded by a flashlight, can wrist cameras take over for manipulation?

🤝 **Practice Exercise**: Before designing your full system, sketch 3 different sensor configurations:
1. **Minimal** (cheapest, fewest sensors)
2. **Robust** (redundancy, multiple modalities)
3. **Performance** (best accuracy, no cost constraints)

Then analyze: What does each configuration optimize for? Which is most appropriate for a real warehouse deployment? This trade-off analysis is the core of perception system design.

## Submission Notes

- **Format**: PDF or Markdown document with diagrams (hand-drawn or digital tools like draw.io, Lucidchart)
- **Length**: Expect 4-6 pages including diagrams and code snippets
- **Collaboration**: Discuss ideas with classmates, but submit your own design
- **Questions**: If you need clarification on warehouse task requirements or sensor specifications, ask in the course forum

This capstone demonstrates your ability to integrate camera systems, depth sensing, IMU data, and sensor fusion into a practical humanoid robot application. Focus on clear justifications and thoughtful trade-off analysis—there's no single "correct" sensor configuration, but your reasoning should show deep understanding of each sensor's strengths and limitations.

Good luck! 🤖✨
