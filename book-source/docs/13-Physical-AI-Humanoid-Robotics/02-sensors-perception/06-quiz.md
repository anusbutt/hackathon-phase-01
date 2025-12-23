---
id: quiz
title: "Module 2 Quiz: Sensors and Perception for Humanoid Robots"
sidebar_position: 6
description: "Assessment quiz covering camera systems, depth sensing, IMU sensors, and sensor fusion for humanoid robots. 15 questions testing comprehension across all four lessons. Passing score: 12/18 points (67%)."
time_estimate: "45 minutes"
difficulty_level: "beginner-intermediate"
prerequisites: ["01-camera-systems", "02-depth-sensing", "03-imu-proprioception", "04-sensor-fusion"]
related_lessons: ["01-camera-systems", "02-depth-sensing", "03-imu-proprioception", "04-sensor-fusion", "05-capstone-project"]
assessment_method: "quiz"
skills:
  - name: "Sensor Comprehension"
    proficiency_level: "beginner-intermediate"
    category: "knowledge-assessment"
    bloom_level: "understand"
    digcomp_area: "technical-concepts"
    measurable_at_this_level: "demonstrate understanding of sensor concepts through quiz performance (12/18 points minimum)"
  - name: "Code Reading (sensor_msgs)"
    proficiency_level: "beginner"
    category: "programming"
    bloom_level: "understand"
    digcomp_area: "programming"
    measurable_at_this_level: "read and explain Python code for sensor_msgs integration"
learning_objectives:
  - objective: "Assess understanding of camera systems, types, and ROS2 integration"
    proficiency_level: "beginner"
    bloom_level: "understand"
    assessment_method: "questions 1-4 (Lesson 1)"
  - objective: "Assess understanding of depth sensing technologies (LiDAR, depth cameras) and ROS2 message formats"
    proficiency_level: "beginner"
    bloom_level: "apply"
    assessment_method: "questions 5-8 (Lesson 2)"
  - objective: "Assess understanding of IMU sensors and proprioception for balance control"
    proficiency_level: "beginner"
    bloom_level: "understand"
    assessment_method: "questions 9-11 (Lesson 3)"
  - objective: "Assess understanding of sensor fusion principles and multi-sensor integration"
    proficiency_level: "intermediate"
    bloom_level: "apply"
    assessment_method: "questions 12-13 (Lesson 4)"
  - objective: "Assess ability to integrate concepts across multiple lessons"
    proficiency_level: "intermediate"
    bloom_level: "apply"
    assessment_method: "questions 14-15 (integration)"
cognitive_load:
  new_concepts: 0
  assessment: "assessment tool - no new concepts introduced, evaluates retention and comprehension from Lessons 1-4"
differentiation:
  extension_for_advanced: "Challenge: Score 16+/18 points (89%+). Then complete hands-on implementation of sensor fusion algorithms using actual ROS2 sensor data."
  remedial_for_struggling: "If scoring below 12/18, review specific lessons indicated in study guide, focus on one lesson at a time, then retake quiz."
tags: ["quiz", "assessment", "sensors", "camera", "lidar", "imu", "sensor-fusion", "evaluation"]
generated_by: "manual"
created: "2025-12-13"
last_modified: "2025-12-13"
ros2_version: "humble"
---

# Module 2 Quiz: Sensors and Perception for Humanoid Robots

Test your understanding of camera systems, depth sensing, IMU sensors, and sensor fusion techniques for humanoid robots.

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
- Questions 1-4: [Lesson 1 - Camera Systems](./camera-systems)
- Questions 5-8: [Lesson 2 - Depth Sensing](./depth-sensing)
- Questions 9-11: [Lesson 3 - IMU & Proprioception](./imu-proprioception)
- Questions 12-13: [Lesson 4 - Sensor Fusion](./sensor-fusion)
- Questions 14-15: Integration across multiple lessons

---

## Questions

### Lesson 1: Camera Systems (Questions 1-4)

**Question 1 (Multiple Choice)** - 1 point

What is the primary difference between monocular, stereo, and RGB-D cameras in terms of depth perception?

A) Monocular cameras can see in 3D, stereo cameras are 2D, RGB-D cameras are color only
B) Monocular cameras provide no depth, stereo cameras estimate depth through triangulation, RGB-D cameras provide direct depth measurements
C) Monocular cameras are for indoor use, stereo cameras for outdoor, RGB-D cameras for underwater
D) Monocular cameras are expensive, stereo cameras are cheap, RGB-D cameras are free

<details>
<summary>Reveal Answer</summary>

**Correct Answer: B**

**Explanation**: Monocular cameras provide only 2D images with no depth information. Stereo cameras use two lenses to capture slightly different views, allowing depth estimation through triangulation. RGB-D cameras (like Kinect) provide both color (RGB) and direct depth measurements (D) from structured light or time-of-flight sensors.

*Lesson Reference: 01-camera-systems.md (Camera Types for Humanoid Robots)*
</details>

---

**Question 2 (Multiple Choice)** - 1 point

Which ROS2 message type is used for standard camera image data?

A) sensor_msgs/Image
B) sensor_msgs/Camera
C) sensor_msgs/Video
D) sensor_msgs/CameraImage

<details>
<summary>Reveal Answer</summary>

**Correct Answer: A**

**Explanation**: The sensor_msgs/Image message type is the standard ROS2 message for camera image data. It contains the raw pixel data along with metadata like encoding, height, width, and step size.

*Lesson Reference: 01-camera-systems.md (ROS2 Integration - sensor_msgs/Image)*
</details>

---

**Question 3 (Short Answer)** - 2 points

Explain the trade-offs between placing cameras on a humanoid robot's head, chest, and wrists. For each location, describe one advantage and one disadvantage for humanoid robot tasks.

<details>
<summary>Reveal Answer & Rubric</summary>

**Sample Answer**:
- **Head**: Advantage - matches human eye perspective for natural interaction and navigation. Disadvantage - limited view of hands during manipulation tasks.
- **Chest**: Advantage - good view of ground and obstacles for navigation. Disadvantage - limited view of upper environment and human faces during interaction.
- **Wrists**: Advantage - provides visual feedback during manipulation tasks. Disadvantage - limited field of view and constantly moving with arm motion.

**Grading Rubric (2 points total)**:
- **2 points**: Correctly identifies advantages and disadvantages for all three locations with specific robot tasks
- **1 point**: Correctly identifies advantages/disadvantages for 1-2 locations OR has partial understanding
- **0 points**: Incorrect or irrelevant responses

*Lesson Reference: 01-camera-systems.md (Camera Placement Strategies)*
</details>

---

**Question 4 (Short Answer)** - 1 point

What does the "field of view" (FOV) parameter indicate for a camera, and why is it important for humanoid robot navigation?

<details>
<summary>Reveal Answer & Rubric</summary>

**Sample Answer**: Field of view indicates the angular extent of the scene that a camera can capture. For humanoid robots, FOV is important because it determines how much of the environment the robot can see at once, affecting obstacle detection, navigation safety, and situational awareness.

**Grading Rubric (1 point)**:
- **1 point**: Correctly explains FOV as angular extent AND relates to robot navigation/environment awareness
- **0 points**: Incorrect or incomplete explanation

*Lesson Reference: 01-camera-systems.md (Camera Parameters - Field of View)*
</details>

---

### Lesson 2: Depth Sensing (Questions 5-8)

**Question 5 (Multiple Choice)** - 1 point

How does 2D LiDAR differ from 3D LiDAR in terms of environmental perception for humanoid robots?

A) 2D LiDAR is for indoor use, 3D LiDAR is for outdoor use
B) 2D LiDAR measures distance in one plane (typically ground level), 3D LiDAR measures in multiple planes (volume)
C) 2D LiDAR is faster, 3D LiDAR is more accurate
D) 2D LiDAR is expensive, 3D LiDAR is cheap

<details>
<summary>Reveal Answer</summary>

**Correct Answer: B**

**Explanation**: 2D LiDAR typically scans in a single horizontal plane and provides distance measurements in that plane only. 3D LiDAR scans in multiple planes to create a 3D point cloud, providing full volumetric information about the environment including height and depth.

*Lesson Reference: 02-depth-sensing.md (LiDAR Technologies - 2D vs 3D)*
</details>

---

**Question 6 (Multiple Choice)** - 1 point

Which ROS2 message type is used for 3D point cloud data from depth sensors?

A) sensor_msgs/PointCloud
B) sensor_msgs/Point3D
C) sensor_msgs/PointCloud2
D) sensor_msgs/LaserScan3D

<details>
<summary>Reveal Answer</summary>

**Correct Answer: C**

**Explanation**: sensor_msgs/PointCloud2 is the standard ROS2 message for 3D point cloud data. It contains structured binary data representing 3D coordinates of points in space, along with optional color and other per-point information.

*Lesson Reference: 02-depth-sensing.md (ROS2 Integration - sensor_msgs/PointCloud2)*
</details>

---

**Question 7 (Short Answer)** - 2 points

Compare the advantages and disadvantages of LiDAR vs stereo vision for humanoid robot navigation. Provide one advantage and one disadvantage for each technology.

<details>
<summary>Reveal Answer & Rubric</summary>

**Sample Answer**:
- **LiDAR Advantages**: Precise distance measurements, works in darkness, reliable in various lighting conditions
- **LiDAR Disadvantages**: Cannot detect transparent surfaces, higher cost, sensitive to rain/fog
- **Stereo Vision Advantages**: Provides color information, lower cost, works in most environments
- **Stereo Vision Disadvantages**: Sensitive to lighting conditions, less accurate at longer distances, computationally intensive

**Grading Rubric (2 points total)**:
- **2 points**: Correctly identifies one advantage and one disadvantage for both technologies
- **1 point**: Correctly identifies advantage/disadvantage for one technology OR partial understanding of both
- **0 points**: Incorrect or irrelevant responses

*Lesson Reference: 02-depth-sensing.md (Depth Technology Comparison)*
</details>

---

**Question 8 (Short Answer)** - 1 point

What is a "point cloud" in the context of depth sensing, and how does it differ from a regular 2D image?

<details>
<summary>Reveal Answer & Rubric</summary>

**Sample Answer**: A point cloud is a collection of 3D points in space, where each point has X, Y, and Z coordinates representing the 3D structure of objects in the environment. Unlike a 2D image which only has X, Y coordinates with color/intensity values, a point cloud provides full 3D spatial information about the scene.

**Grading Rubric (1 point)**:
- **1 point**: Correctly explains point cloud as 3D points with X, Y, Z coordinates AND differentiates from 2D images
- **0 points**: Incorrect or incomplete explanation

*Lesson Reference: 02-depth-sensing.md (Point Cloud Fundamentals)*
</details>

---

### Lesson 3: IMU and Proprioception (Questions 9-11)

**Question 9 (Multiple Choice)** - 1 point

Which three sensors are typically combined in an IMU for humanoid robot balance?

A) Camera, gyroscope, magnetometer
B) Accelerometer, gyroscope, magnetometer
C) LiDAR, accelerometer, gyroscope
D) GPS, magnetometer, camera

<details>
<summary>Reveal Answer</summary>

**Correct Answer: B**

**Explanation**: An IMU (Inertial Measurement Unit) combines three sensor types: accelerometer (measures linear acceleration and gravity), gyroscope (measures angular velocity/rotation rates), and magnetometer (measures magnetic field for heading). Together they provide comprehensive motion and orientation sensing.

*Lesson Reference: 03-imu-proprioception.md (What Is an IMU?)*
</details>

---

**Question 10 (Multiple Choice)** - 1 point

What does a gyroscope in an IMU measure for humanoid robot balance?

A) Linear acceleration and gravity
B) Angular velocity (rate of rotation)
C) Magnetic field direction
D) Distance to obstacles

<details>
<summary>Reveal Answer</summary>

**Correct Answer: B**

**Explanation**: A gyroscope measures angular velocity - the rate at which the robot is rotating around any axis. This is crucial for balance control as it allows the robot to detect when it's starting to tip or rotate unexpectedly.

*Lesson Reference: 03-imu-proprioception.md (Gyroscope Function)*
</details>

---

**Question 11 (Short Answer)** - 2 points

Explain how IMU data can be used to detect if a humanoid robot is falling. Describe the sensor readings that would indicate a fall is occurring.

<details>
<summary>Reveal Answer & Rubric</summary>

**Sample Answer**: A humanoid robot falling would show characteristic IMU readings: (1) Accelerometer readings would show near-zero linear acceleration during freefall (as the robot and sensors accelerate together at gravity), followed by high acceleration during impact. (2) Gyroscope readings might show rapid, uncontrolled rotation during the fall. (3) The combination of these readings can be processed by fall detection algorithms that look for acceleration magnitude thresholds and rotation patterns that indicate loss of balance.

**Grading Rubric (2 points total)**:
- **2 points**: Correctly explains freefall acceleration near zero AND impact acceleration AND mentions gyroscope rotation patterns
- **1 point**: Correctly explains one or two aspects of fall detection
- **0 points**: Incorrect or irrelevant explanation

*Lesson Reference: 03-imu-proprioception.md (Fall Detection with IMU)*
</details>

---

### Lesson 4: Sensor Fusion (Questions 12-13)

**Question 12 (Multiple Choice)** - 1 point

Why is sensor fusion essential for humanoid robot perception?

A) It reduces the cost of individual sensors
B) It combines multiple sensors to overcome individual limitations and create more robust perception
C) It increases the size of the robot
D) It eliminates the need for any single sensor

<details>
<summary>Reveal Answer</summary>

**Correct Answer: B**

**Explanation**: Sensor fusion combines data from multiple sensors to create more accurate, reliable, and complete information than any single sensor could provide. It allows robots to overcome limitations (e.g., cameras fail in darkness, LiDAR struggles with transparent surfaces) by intelligently integrating complementary strengths.

*Lesson Reference: 04-sensor-fusion.md (What Is Sensor Fusion?)*
</details>

---

**Question 13 (Multiple Choice)** - 1 point

What does a complementary filter do in sensor fusion?

A) It eliminates all sensor data
B) It combines gyroscope and accelerometer data with configurable time constants to provide drift-corrected orientation
C) It only uses camera data
D) It doubles the sensor data

<details>
<summary>Reveal Answer</summary>

**Correct Answer: B**

**Explanation**: A complementary filter combines gyroscope data (good for short-term rotation tracking) with accelerometer data (good for long-term orientation reference) using configurable time constants. This provides drift-corrected orientation estimation by leveraging the strengths of each sensor type.

*Lesson Reference: 04-sensor-fusion.md (Complementary Filtering)*
</details>

---

### Integration Questions (Questions 14-15)

**Question 14 (Short Answer)** - 2 points

Design a minimal multi-sensor perception system for a humanoid robot that needs to navigate indoors, avoid obstacles, and maintain balance. Identify which sensors from Module 2 would be essential and explain why each is necessary.

<details>
<summary>Reveal Answer & Rubric</summary>

**Sample Answer**: Essential sensors would include: (1) Stereo cameras for navigation scene understanding and human interaction, (2) 3D LiDAR for precise obstacle detection and mapping (works in darkness), (3) IMU for balance control and orientation estimation. Cameras provide visual context, LiDAR provides reliable distance measurements for navigation, and IMU provides internal motion sensing for balance. The fusion of all three creates robust perception that works in various lighting conditions and provides both external awareness and internal state knowledge.

**Grading Rubric (2 points total)**:
- **2 points**: Identifies at least 2-3 appropriate sensors AND explains specific reasons for each in the context of the task
- **1 point**: Identifies sensors but with limited explanation OR identifies only 1-2 sensors with good explanation
- **0 points**: Incorrect sensor choices or irrelevant explanations

*Lesson Reference: 04-sensor-fusion.md (Multi-Sensor Integration)*
</details>

---

**Question 15 (Short Answer)** - 2 points

Explain how a humanoid robot would use sensor fusion to navigate safely when transitioning from a well-lit indoor area to a dark hallway. What sensors would become more or less important in each environment?

<details>
<summary>Reveal Answer & Rubric</summary>

**Sample Answer**: In the well-lit area, cameras would provide excellent navigation information for obstacle detection and path planning. As the robot enters darkness, camera effectiveness decreases significantly. The fusion system would automatically reduce weighting of camera data while increasing reliance on LiDAR and depth sensors, which work independently of lighting. The IMU would continue providing balance and orientation information throughout the transition. The robot's navigation system would adaptively combine available sensor data based on confidence levels, maintaining safe navigation throughout the lighting transition.

**Grading Rubric (2 points total)**:
- **2 points**: Correctly explains sensor adaptation during lighting transition AND identifies which sensors become more/less important
- **1 point**: Explains some aspects of sensor adaptation OR identifies some sensor changes
- **0 points**: Incorrect or irrelevant explanation

*Lesson Reference: 04-sensor-fusion.md (Adaptive Sensor Weighting)*
</details>

---