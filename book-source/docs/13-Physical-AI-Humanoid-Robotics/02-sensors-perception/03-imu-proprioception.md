---
title: "Lesson 3: IMU and Proprioception for Humanoid Robots"
sidebar_label: "IMU & Proprioception"
sidebar_position: 3
description: "Learn how inertial measurement units and proprioceptive sensors enable humanoid robots to sense their own motion, orientation, and body position for dynamic balance control and spatial self-awareness."
keywords:
  - IMU
  - inertial measurement unit
  - proprioception
  - accelerometer
  - gyroscope
  - sensor fusion
  - ROS2
  - sensor_msgs/Imu
tags:
  - ros2
  - sensors
  - humanoid-robotics
  - perception
difficulty: "Beginner-Intermediate"
estimated_time: "45-55 minutes"
prerequisites:
  - "Module 1: ROS2 foundations (nodes, topics, messages)"
  - "Lesson 1: Camera fundamentals and computer vision basics"
  - "Lesson 2: Depth sensing technologies"
  - "Understanding of coordinate frames and transformations"
  - "Basic physics concepts (acceleration, angular velocity, gravity)"
learning_objectives:
  - "Understand how IMU sensors (accelerometer, gyroscope, magnetometer) measure motion and orientation for humanoid balance control"
  - "Apply knowledge of sensor_msgs/Imu message structure to subscribe to and process inertial data in ROS2"
  - "Analyze the trade-offs between sensor drift, noise, and long-term accuracy for accelerometers, gyroscopes, and magnetometers"
  - "Evaluate proprioceptive sensor integration (IMU + joint encoders) for whole-body state awareness in humanoid robots"
  - "Create balance monitoring and fall detection systems using real-time IMU feedback loops"
skills:
  - "Subscribe to sensor_msgs/Imu topics and extract orientation quaternions and acceleration vectors"
  - "Convert quaternions to Euler angles (roll, pitch, yaw) for tilt detection and balance monitoring"
  - "Implement fall detection algorithms using acceleration magnitude thresholds for freefall and impact phases"
  - "Design closed-loop balance control systems with ankle, hip, and stepping strategies based on IMU feedback"
  - "Integrate IMU data with joint encoder information for complete proprioceptive body awareness"
toc_min_heading_level: 2
toc_max_heading_level: 4
---

# Lesson 3: IMU and Proprioception for Humanoid Robots

## What Is an IMU?

An IMU (Inertial Measurement Unit) is a sensor that measures motion and orientation without relying on external reference points like GPS satellites or visual landmarks. It detects acceleration, rotation rates, and magnetic field direction, giving robots an "internal sense" of movement that works anywhere - underground, indoors, outdoors, even in complete darkness.

An IMU combines three distinct types of sensors working together. The **accelerometer** measures linear acceleration (including gravity) in meters per second squared (m/s²), allowing the robot to detect how fast it's speeding up, slowing down, or tilting. The **gyroscope** detects angular velocity - the rate of rotation - in radians per second (rad/s), tracking how quickly the robot is turning around any axis. The **magnetometer** senses Earth's magnetic field to establish heading or compass direction, much like a traditional compass but in digital form. Together, these three sensors create a complete picture of the robot's motion and orientation without needing to "see" anything.

The term "inertial" comes from the physics principle of inertia - how objects resist changes in motion. Because IMU measurements are based on detecting changes in motion rather than observing external landmarks, they work in environments where cameras and depth sensors fail. You can navigate with an IMU in a tunnel with no GPS signal, in fog where cameras can't see, or in darkness where visual sensors are useless.

For humanoid robots, IMUs are absolutely critical. A humanoid robot must balance on two feet like a human, which is inherently unstable - imagine standing on stilts while someone pushes you. The IMU provides constant, real-time feedback about the robot's body orientation and motion, enabling bipedal locomotion, dynamic stability, and recovery from disturbances. Without an IMU, the robot simply cannot know if it's tilting forward, backward, or sideways until it's already falling. By the time a camera detects the ground rushing up, it's too late to react. The IMU is the robot's inner ear - the sense that keeps it upright.

## Why IMU Matters for Physical AI

Humanoid robots stand on two feet, making them inherently unstable in the same way humans are. Unlike wheeled robots with their stable four-point contact, bipedal robots are constantly on the edge of falling. Without IMU feedback, the robot is effectively "blind" to its own tilt, rotation, and acceleration. Camera-only perception introduces processing lag - by the time the visual system detects a problem, the robot may already be falling. When a humanoid starts to tip, it must react in milliseconds, far too fast for visual processing alone. This is where the IMU becomes indispensable.

**Self-Awareness and Proprioception**: An IMU gives the robot knowledge of its own orientation even in total darkness or thick fog. Combined with joint encoders that report the angle of each limb, the robot knows exactly where its body is positioned in three-dimensional space. This is analogous to human proprioception - you know where your arm is even with your eyes closed. This internal body awareness is foundational for manipulation tasks (reaching for objects while maintaining balance), posture control (adjusting stance on uneven surfaces), and graceful falling (detecting an unrecoverable tip and positioning limbs to minimize damage).

**Dynamic Balance and Fall Prevention**: The IMU detects tilt before it becomes a catastrophic fall. By monitoring orientation changes in real time, the robot can execute corrective movements through a control loop that operates in 25-50 milliseconds from disturbance detection to motor actuation, with complete corrective movements executing within 50-200 milliseconds - faster than human reaction time of approximately 200 milliseconds. This allows the implementation of balance strategies like ankle adjustments (rotating feet to shift center of pressure), hip movements (bending at the waist to counterbalance), or taking a recovery step. The difference between a robot that stumbles awkwardly and one that recovers gracefully comes down to fast, accurate IMU feedback. In real-world scenarios - a robot operating on a construction site, navigating a crowded sidewalk, or responding to an unexpected push - this capability is essential for safety and reliability.

**Motion Estimation and Localization**: When visual sensors fail or become unreliable, the IMU provides continuous motion feedback. In GPS-denied environments like underground facilities, indoor warehouses, or tunnels, the IMU can track body motion between visual landmarks. It complements camera-based localization by filling the gaps when lighting conditions are poor, the environment is textureless (like a blank white hallway), or the robot is moving too fast for visual processing. While cameras tell the robot where it is relative to the world, the IMU tells it how it's moving right now.

Scaling this to humanoid tasks reveals the full importance. Picking up objects requires balance awareness while reaching - you lean forward to grab something, the IMU detects the shift in center of mass, and the balance controller adjusts your stance to prevent tipping. Walking on uneven surfaces demands constant IMU feedback to drive foot placement decisions - detect the tilt from stepping on a rock, adjust the next step accordingly. Human-robot interaction scenarios involve reacting to external pushes or pulls, which the IMU detects instantly. Future humanoid robots performing sports-like activities - jumping, running, catching - will rely heavily on high-frequency IMU data to coordinate dynamic movements. The IMU is the foundation for robots doing real human tasks in human spaces.

## Key Principles

### Principle 1: IMU Sensor Components and What They Measure

Each of the three sensors in an IMU measures a different aspect of motion, and understanding what each one provides is key to interpreting IMU data correctly.

The **accelerometer** measures linear acceleration in three axes (X, Y, Z). Its units are meters per second squared (m/s²). Critically, it always measures gravity - when the robot is sitting perfectly still on a table, the accelerometer reads approximately 9.81 m/s² pointing upward (counteracting Earth's downward gravitational pull). This makes it excellent for detecting tilt: if the robot leans forward, the direction of the gravity vector changes relative to the robot's body frame. Accelerometers have high noise in their readings due to vibrations and sensor imperfections, but they provide a reliable long-term reference because gravity is constant. They're used for tilt detection, fall detection (sudden acceleration spikes), and tracking linear motion.

The **gyroscope** measures angular velocity - how fast the robot is rotating around each of its three axes. Units are radians per second (rad/s) or degrees per second (deg/s). For example, turning your head left produces angular velocity around the vertical axis; tilting your head forward produces angular velocity around the horizontal axis. Gyroscopes are very accurate for short time periods (seconds to minutes) with low noise, making them ideal for detecting rapid rotation. However, they suffer from **drift** - small measurement errors accumulate over time. A gyroscope with a drift rate of 1 degree per second will be off by 3600 degrees after one hour of operation. This means gyroscopes are excellent for fast orientation changes but need constant recalibration from other sensors for long-term accuracy.

The **magnetometer** measures the strength and direction of Earth's magnetic field in three axes. Units are microteslas (μT). Unlike the gyroscope, which measures changes in orientation, the magnetometer provides an absolute heading reference - it always knows which direction is magnetic north. This makes it invaluable for correcting gyroscope drift over long periods. However, magnetometers are highly sensitive to interference from electronics, metal objects, and power lines, making them unreliable indoors or near machinery. They also typically have slower update rates compared to accelerometers and gyroscopes.

### Principle 2: Sensor Drift and Noise Characteristics

No sensor is perfect. Each component of the IMU has distinct error characteristics that influence how we use it.

Accelerometers have the advantage of being reliable over long time periods. Because they measure gravity, which never changes, they provide a consistent reference for "down" that doesn't drift over hours or days. However, they are noisy - vibrations from motors, footsteps, or even wind can add high-frequency noise to the readings. They also have bias errors, systematic offsets from the true value, though these can often be calibrated out during initialization. The use case for accelerometers is detecting tilt with high accuracy over extended periods, making them the foundation for long-term orientation estimation.

Gyroscopes are highly accurate in the short term with very low noise, making them perfect for tracking fast rotational movements. But their fatal flaw is drift. Consumer-grade MEMS gyroscopes typically have drift rates of 0.1 to 1 degree per hour, while tactical-grade IMUs used in humanoid robots achieve 1-3 degrees per hour, and navigation-grade systems can reach below 0.01 degrees per hour. Even at these rates, drift accumulates - a 1 degree per hour drift means the orientation estimate will be off by 24 degrees after one day of operation. This is why gyroscopes must be continuously corrected using other sensors. Their use case is measuring fast orientation changes over timescales from milliseconds to hours, after which they need recalibration.

Magnetometers provide an absolute heading reference, which is their key strength. No matter how much the robot spins, the magnetometer can tell you which direction is magnetic north. However, they're plagued by magnetic interference. A nearby motor, a steel beam, or even the robot's own electronics can distort the readings. Indoors, magnetometers are often unreliable. They also have slower update rates than accelerometers and gyroscopes, making them unsuitable for fast control loops.

The key insight is that no single sensor is perfect. Accelerometers drift less but are noisy. Gyroscopes are precise but drift quickly. Magnetometers provide absolute reference but are interference-prone. This is why sensor fusion - intelligently combining all three - is essential for robust orientation estimation, which we'll explore in the next lesson.

### Principle 3: sensor_msgs/Imu Message Structure and ROS2 Integration

In ROS2, IMU data flows through the system using the standard `sensor_msgs/Imu` message type. Understanding its structure is critical for working with IMU data in your robot code.

The message contains six main fields. The **header** includes a timestamp (when the measurement was taken) and a frame_id (which robot link the IMU is physically attached to, like "base_link" or "imu_link"). This allows the system to transform IMU data into other coordinate frames if needed. The **orientation** field is a quaternion - a four-value representation of 3D rotation as [x, y, z, w]. Quaternions are used instead of Euler angles (roll, pitch, yaw) because they avoid gimbal lock, a mathematical singularity that occurs at certain orientations. For example, a quaternion of [0, 0, 0.707, 0.707] represents a 90-degree rotation around the Z-axis. While quaternions are compact and mathematically robust, they're not intuitive, so they're often converted to Euler angles for human interpretation or simple control logic.

The **angular_velocity** field is a Vector3 containing the rotation rate around the X, Y, and Z axes in radians per second. This comes directly from the gyroscope. The **linear_acceleration** field is another Vector3 with acceleration along the X, Y, and Z axes in meters per second squared, coming from the accelerometer. Remember that this includes gravity - a stationary robot will show [0, 0, 9.81] if Z points upward.

The message also includes **covariance matrices** - 9-element arrays (representing 3x3 matrices in row-major order) for orientation, angular velocity, and linear acceleration. These covariance values encode the uncertainty or confidence in each measurement. High covariance means low confidence; low covariance means high confidence. Sensor fusion algorithms use these values to weight how much to trust each sensor at any given moment.

In practice, the flow works like this: The physical IMU sensor captures raw data, which the ROS2 driver packages into a `sensor_msgs/Imu` message and publishes to a topic (commonly `/imu/data`). Your robot's control software subscribes to this topic and receives updates at 100-500 Hz depending on the hardware. From this data, you can compute tilt angles from the orientation quaternion, detect falling by monitoring acceleration spikes, or track rotation by integrating angular velocity.

For example, a stationary robot on level ground will show `linear_acceleration = [0, 0, 9.81]` (only gravity), `angular_velocity = [0, 0, 0]` (not rotating), and an orientation quaternion close to [0, 0, 0, 1] (no rotation from the initial pose). If the robot starts tilting forward, the acceleration vector rotates - the Z component decreases while the X component (forward direction) increases. If the robot spins, the angular_velocity around the Z-axis becomes non-zero.

### Principle 4: Proprioception - Robot's Sense of Body Position and Motion

Proprioception is the sense of knowing where your body is and how it's moving without looking at it. Close your eyes and raise your arm above your head - you know exactly where your arm is. That's proprioception, enabled by nerve endings in your muscles and joints. Robots need the same capability.

In robotics, proprioception is the ability to sense the robot's own body position and motion without external observation. While humans use biological sensors embedded in tissues, robots achieve proprioception through a combination of hardware sensors. IMU data provides the orientation and acceleration of the robot's torso or base. Joint encoders report the angle at each articulated joint - shoulder, elbow, hip, knee, ankle. Together, these sensors create a complete real-time picture of the robot's body state in three dimensions.

This internal body awareness is critical for humanoid robots in ways that wheeled robots don't experience. During walking, the robot needs to know foot placement relative to its center of mass to maintain balance. While reaching for an object, it must know arm position relative to its moving torso. When reacting to an external push, the robot needs instantaneous feedback about body tilt to adjust muscles and prevent falling. Unlike wheeled robots, which have stable bases and relatively simple kinematics, humanoid bipeds are constantly adjusting for stability, making proprioception a moment-to-moment necessity.

The distinction between proprioception and vision is important. Vision provides external perception - information about the world around the robot. It can fail in darkness, fog, or when the camera is occluded. It also operates at relatively low frequencies (typically 30 Hz for cameras) with processing delays. Proprioception, by contrast, is internal perception - information about the robot's own body. It's always available regardless of lighting or environmental conditions, and it operates at high frequencies (100+ Hz for IMUs, 1000+ Hz for joint encoders) with minimal latency. Both are needed: vision for navigation and understanding the environment, proprioception for balance and body control.

For computer science students, a useful analogy is how a programming language maintains internal state. Your program knows the values of its variables without needing to query an external database. Similarly, the robot maintains state of its own body through proprioceptive sensors - it knows where it is without needing to "look at itself" through cameras.

### Principle 5: Balance Control Loop - Feedback and Correction

Balance for a humanoid robot is not a passive state - it's an active control loop running continuously at high speed. The IMU is the sensor that makes this loop possible.

The closed-loop balance system has four stages that repeat constantly. First, **sense**: The IMU detects the current tilt angle (using the accelerometer's gravity reference) and rotation rate (from the gyroscope). Second, **compute**: The balance controller analyzes the tilt and rate of change to determine what corrective action is needed. Third, **act**: The robot executes the corrective strategy by adjusting motor commands to ankle, hip, or leg joints. Fourth, the loop repeats, typically every 10-50 milliseconds.

Roboticists use three primary balance strategies, escalating in complexity based on the severity of the disturbance. The **ankle strategy** handles small disturbances, typically with tilt less than approximately 5 degrees (exact thresholds vary by robot design and perturbation speed). The robot rotates its ankles to move the center of pressure (where force is applied to the ground) under the center of mass (where gravity acts on the body). This is the fastest and most energy-efficient strategy. For example, if the IMU detects a 2-degree forward tilt, the controller commands the ankles to rotate backward slightly, bringing the robot back to vertical.

The **hip strategy** activates for moderate disturbances with tilt between 5 and 15 degrees. The robot bends at the hips while keeping the ankles relatively fixed, using larger body movements to shift the center of mass. This is more powerful than ankle adjustments but takes slightly longer and consumes more energy. If the IMU detects an 8-degree tilt that exceeds ankle strategy limits, the robot bends forward at the hips to counterbalance.

The **stepping strategy** is the last resort for large disturbances exceeding 15 degrees. The robot takes a step to place a foot under the new projected center of mass, effectively catching itself before falling. This is the most complex strategy, requiring coordination of swing leg trajectory, balance transfer, and landing control. If the IMU predicts based on tilt angle and angular velocity that the robot will fall, it initiates a recovery step.

The timing of this feedback loop is critical. A typical IMU updates at 200 Hz, meaning new data arrives every 5 milliseconds. The controller processes this data and computes a response in another 5-20 milliseconds. Motor commands are sent and begin executing within milliseconds. The total reaction time from disturbance detection to corrective action is 25-50 milliseconds. This is actually faster than human reaction time, which is around 200 milliseconds. The speed of the loop is what allows robots to maintain balance on two feet - without it, they'd topple before they could react.

### 💬 AI Colearning Prompt

> **Sensor Fusion Challenge**: Ask Claude to explain how you would combine accelerometer, gyroscope, and magnetometer readings to get a reliable orientation estimate that doesn't drift over time. What information does each sensor provide that others don't? Where might conflicts arise between sensor readings, and how would you resolve them?

### 🎓 Expert Insight: IMU Initialization, Calibration, and Why Your Robot Falls Over

Many students assume that an IMU provides perfect orientation data the moment you power it on. The reality is quite different, and understanding this gap can save you hours of debugging when your robot unexpectedly falls over.

IMUs require a calibration and initialization phase before they can provide reliable data. The gyroscope, in particular, suffers from a systematic offset called bias - even when perfectly stationary, it reports a small non-zero angular velocity. This bias must be measured at startup while the robot is completely still. If you skip this step, your orientation estimate will be wrong from the very first reading, accumulating error immediately. Magnetometers also need calibration to account for local magnetic distortions. The calibration process typically involves keeping the robot stationary for 2-5 seconds to measure gyroscope bias, then slowly rotating the robot through its full range of motion to map magnetometer readings to known orientations. Higher-end IMU chips often have built-in self-calibration routines, but cheaper sensors require manual calibration in your initialization code. The impact of skipping this step is dramatic - the difference between stable walking and immediate falling.

That covariance matrix in the `sensor_msgs/Imu` message is not just metadata you can ignore. It tells your sensor fusion algorithm how much to trust each reading at this exact moment. A good IMU under normal conditions reports low covariance, indicating high confidence in the measurements. A faulty IMU, or one experiencing interference or vibration, will report high covariance. Production robotic systems monitor this covariance continuously. If it suddenly spikes, the system knows something is wrong - maybe the IMU is malfunctioning, maybe there's unexpected magnetic interference - and it can switch to backup sensors or enter a safe mode rather than trusting bad data and falling.

This is precisely why sensor fusion exists, which we'll cover in the next lesson. No single sensor is reliable enough for critical balance control. By combining three or more sensors and monitoring their covariance matrices, you create a robust system that tolerates individual sensor faults. The IMU is foundational, but it's just one piece of the perception puzzle.

## Real-World Examples

Understanding IMU specifications and applications in theory is valuable, but examining how leading robotics companies implement these sensors in production humanoid robots provides crucial practical insights. The following case studies demonstrate different IMU configurations, sensor fusion strategies, and integration approaches that enable balance, proprioception, and dynamic motion control in real-world humanoid platforms.

### 📊 Boston Dynamics Atlas

Boston Dynamics' Atlas represents the pinnacle of humanoid robotics, utilizing a sophisticated IMU system for balance and dynamic control. The robot employs a **KVH 1750 tactical-grade IMU** composed of Fiber Optic Gyroscopes (FOG) combined with MEMS accelerometers. This IMU is strategically positioned in the robot's pelvis, located 9 cm behind the pelvis link and rotated 45° for optimal measurement orientation.

The KVH 1750 IMU features programmable data output rates from **1-1000 Hz** and is available with **2g, 10g, or 30g accelerometer ranges**, with Atlas likely using higher ranges to handle dynamic movements like parkour and backflips. The fiber optic gyroscopes provide superior angular momentum measurements compared to MEMS-only solutions, crucial for Atlas's acrobatic capabilities.

**Key Technical Insight**: Atlas's choice of FOG-based IMU over pure MEMS reflects a design priority for precision over cost. The pelvis placement centralizes inertial measurements at the robot's center of mass, while sensor fusion with force sensors, LiDAR, stereo vision, and joint encoders enables the Extended Kalman Filter (EKF) algorithms that maintain balance stability above 95% even during high-impact landings.

### 📊 Agility Robotics Digit

Digit, designed for warehouse logistics and bipedal manipulation tasks, uses a **MEMS IMU** integrated with a comprehensive sensor suite including four Intel RealSense depth cameras, LiDAR, force sensors in the arms, and both absolute and incremental encoders. The robot stands 5'9" (175 cm) tall with 28 degrees of freedom and operates for 4 hours on battery power.

Digit's IMU operates at typical MEMS sampling rates (**100-200 Hz for accelerometers, 200-500 Hz for gyroscopes**) with estimated ranges of **±4g accelerometer** and **±500°/s gyroscope** based on industry standards for bipedal robots. The torso-mounted IMU feeds real-time inertial data into sophisticated whole-body control hierarchies that compute Zero Moment Point (ZMP) and Center of Pressure (CoP) calculations for locomotion stability.

**Key Technical Insight**: Digit's sensor fusion approach demonstrates how combining IMU data with force sensor feedback and geometric models through EKF algorithms creates robust balance control. The integration of IMU with depth cameras and encoders significantly improves overall pose estimation accuracy compared to encoder-only approaches. The robot was trained across billions of simulation steps to enhance stability management when encountering physical disturbances, demonstrating how IMU data quality directly impacts machine learning model performance.

### 📊 Unitree H1

The Unitree H1 holds the Guinness World Record for the fastest full-sized humanoid robot at **3.3 m/s** and was the first electric humanoid to successfully land a standing backflip—achievements made possible by advanced IMU-based balance control. The H1 combines **real-time IMU sensors** with 3D LiDAR (Livox Mid-360), Intel RealSense D435 depth cameras, joint encoders, and contact sensors in a comprehensive sensor fusion architecture.

At 180 cm tall and 47 kg, the H1 utilizes high-torque actuators (up to **360 Nm joint torque**) controlled by real-time sensor fusion algorithms that process IMU data for dynamic balance recovery. The IMU enables the robot to adapt to rough terrains, recover from external pushes, and maintain stability at high speeds—critical for both the record-breaking running speed and backflip maneuvers.

**Key Technical Insight**: H1's success demonstrates the importance of **response time** in IMU-based balance systems. The sensor fusion loop processes IMU, encoder, and contact data with response times under 10 milliseconds, enabling the rapid corrective joint torques necessary for dynamic stability. The choice to integrate IMU data with 360° depth sensing through LiDAR creates a proprioceptive-exteroceptive feedback loop: IMU measures internal body state while LiDAR maps terrain, allowing predictive balance adjustments before ground contact—essential for high-speed locomotion on uneven surfaces.

## Practical Example: Balance State Monitoring

Imagine you're integrating balance control onto a humanoid robot. The IMU publishes orientation and acceleration data at 200 Hz. Your task is to monitor this stream and detect when the robot enters an unstable state. Specifically, you need to:

1. Subscribe to the IMU topic using the `sensor_msgs/Imu` message type
2. Extract the tilt angle (roll and pitch) from the quaternion orientation
3. Check if tilt exceeds 10 degrees - if so, log a "balance correction needed" warning
4. Track acceleration to detect potential falls

This represents the first stage of a balance control system: sensing and identifying unstable states before they become unrecoverable falls.

```python
"""
IMU subscriber demonstrating balance state monitoring for humanoid robots.
This node detects when the robot tilts beyond safe thresholds.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class BalanceMonitor(Node):
    """Monitors IMU data and detects balance instability."""

    def __init__(self) -> None:
        super().__init__('balance_monitor')
        # Subscribe to IMU data from the robot
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        self.tilt_threshold: float = 10.0  # degrees - ankle strategy limit

    def imu_callback(self, msg: Imu) -> None:
        """Process incoming IMU data and check stability."""
        # Extract quaternion (orientation)
        x, y, z, w = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w

        # Convert quaternion to roll/pitch angles using trigonometric formulas
        roll = math.atan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y))
        pitch = math.asin(2.0*(w*y - z*x))

        # Convert radians to degrees for human-readable output
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)

        # Check if tilt exceeds threshold
        if abs(roll_deg) > self.tilt_threshold or abs(pitch_deg) > self.tilt_threshold:
            self.get_logger().warn(
                f'Imbalance detected! Roll: {roll_deg:.1f}°, Pitch: {pitch_deg:.1f}°'
            )

# Note: Full node lifecycle (rclpy.init, spin, shutdown) omitted for clarity
```

The `BalanceMonitor` node subscribes to the `/imu/data` topic, which publishes `sensor_msgs/Imu` messages at high frequency (typically 200 Hz). In the callback function, we extract the quaternion - a compact four-value representation of 3D rotation that avoids mathematical singularities. We then convert this quaternion to roll and pitch angles using trigonometric formulas. Roll represents rotation around the forward-back axis (leaning left or right), while pitch represents rotation around the left-right axis (leaning forward or backward).

We compare these angles against a 10-degree threshold, which represents the practical limit for the ankle strategy we discussed earlier. If the robot tilts more than 10 degrees, ankle adjustments alone won't be sufficient - the system needs to escalate to hip strategy or stepping strategy. The logger warning simulates sending a signal to the lower-level balance controller to take corrective action. In a complete system, this warning would trigger the appropriate balance strategy based on the severity and rate of tilt.

This example demonstrates the sensing-to-decision pipeline that underlies all robotic control: IMU hardware measures orientation → ROS2 middleware delivers the data as messages → callback function processes the data → control decisions are made based on thresholds and logic. Real production systems add filtering to remove noise, sensor fusion to combine IMU with other sensors, and sophisticated control algorithms that consider velocity and acceleration trends, but this captures the essential pattern. You're seeing Principles 3 and 5 in action: interpreting the `sensor_msgs/Imu` structure and closing the balance control feedback loop.

## Practical Example: Fall Detection Algorithm

Building on the balance monitoring concepts, let's examine a more sophisticated application: detecting when a humanoid robot is experiencing a fall event. Falls have two characteristic phases that IMU sensors can detect: **freefall** (when the robot loses contact with the ground and experiences near-zero acceleration) and **impact** (when the robot hits the ground with a sudden acceleration spike). Detecting these events allows the robot to trigger protective responses like activating compliance control, stopping dangerous motions, or alerting human operators.

```python
# Example 2: Fall Detection Algorithm

"""
Fall Detection Node using IMU Data

This node implements a fall detection algorithm for humanoid robots by monitoring
IMU acceleration data. It detects two phases of a fall:
1. Freefall: Low acceleration magnitude (robot is falling freely)
2. Impact: High acceleration spike (robot hits the ground)

This is crucial for humanoid robots to detect falls and trigger protective responses
like activating compliance control or stopping dangerous motions.

Author: ROS2 Nervous System Tutorial
License: MIT
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from typing import Optional
from rclpy.time import Time


class FallDetector(Node):
    """
    A ROS2 node that detects potential falls using IMU acceleration data.

    The algorithm uses acceleration magnitude thresholds to identify freefall
    and impact events, which are the characteristic phases of a fall.
    """

    def __init__(self) -> None:
        """Initialize the fall detection node with threshold parameters."""
        super().__init__('fall_detector')

        # Fall detection thresholds
        self.FREEFALL_THRESHOLD: float = 2.0  # m/s² - below this indicates freefall
        self.IMPACT_THRESHOLD: float = 20.0   # m/s² - above this indicates impact
        self.FREEFALL_DURATION: float = 0.3   # seconds - minimum freefall time

        # State tracking variables
        self.last_acceleration_magnitude: float = 9.81  # Start with gravity
        self.in_freefall: bool = False
        self.freefall_start_time: Optional[Time] = None
        self.fall_detected: bool = False

        # Create subscription to IMU data
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        self.get_logger().info(
            f'Fall Detector initialized.\n'
            f'Freefall threshold: {self.FREEFALL_THRESHOLD} m/s²\n'
            f'Impact threshold: {self.IMPACT_THRESHOLD} m/s²\n'
            f'Minimum freefall duration: {self.FREEFALL_DURATION} s'
        )

    def calculate_acceleration_magnitude(self, msg: Imu) -> float:
        """
        Calculate the total acceleration magnitude from IMU data.

        The magnitude is computed as: sqrt(ax² + ay² + az²)
        During normal standing, this should be approximately 9.81 m/s² (gravity).

        Args:
            msg: The IMU message containing acceleration data

        Returns:
            The acceleration magnitude in m/s²
        """
        try:
            accel_vector = np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ])

            # Calculate Euclidean norm (magnitude)
            magnitude: float = float(np.linalg.norm(accel_vector))
            return magnitude

        except Exception as e:
            self.get_logger().error(f'Error calculating acceleration magnitude: {e}')
            return 9.81  # Return gravity as default

    def check_freefall(self, magnitude: float, current_time: Time) -> None:
        """
        Check if the robot is in freefall based on acceleration magnitude.

        Freefall is characterized by very low acceleration (near zero) because
        the sensor experiences weightlessness when falling freely.

        Args:
            magnitude: Current acceleration magnitude in m/s²
            current_time: Current ROS2 timestamp
        """
        if magnitude < self.FREEFALL_THRESHOLD:
            if not self.in_freefall:
                # Start of freefall detected
                self.in_freefall = True
                self.freefall_start_time = current_time
                self.get_logger().warn(
                    f'⚠️  FREEFALL DETECTED! Acceleration: {magnitude:.2f} m/s²'
                )
            else:
                # Check if freefall has lasted long enough
                if self.freefall_start_time is not None:
                    freefall_duration = (current_time - self.freefall_start_time).nanoseconds / 1e9
                    if freefall_duration > self.FREEFALL_DURATION:
                        self.get_logger().warn(
                            f'⚠️  SUSTAINED FREEFALL: {freefall_duration:.2f}s - Possible fall in progress!'
                        )
        else:
            # Exit freefall state
            if self.in_freefall:
                self.in_freefall = False
                self.freefall_start_time = None

    def check_impact(self, magnitude: float) -> None:
        """
        Check for sudden impact based on high acceleration spikes.

        When a robot hits the ground, the IMU experiences a sudden, large
        acceleration spike significantly above normal gravity.

        Args:
            magnitude: Current acceleration magnitude in m/s²
        """
        if magnitude > self.IMPACT_THRESHOLD:
            self.get_logger().error(
                f'🚨 IMPACT DETECTED! Acceleration spike: {magnitude:.2f} m/s² - FALL EVENT!'
            )
            self.fall_detected = True

            # Reset freefall tracking after impact
            self.in_freefall = False
            self.freefall_start_time = None

    def imu_callback(self, msg: Imu) -> None:
        """
        Process IMU data to detect fall events.

        Args:
            msg: The IMU message containing sensor measurements
        """
        try:
            # Get current timestamp
            current_time = Time.from_msg(msg.header.stamp)

            # Calculate total acceleration magnitude
            magnitude = self.calculate_acceleration_magnitude(msg)

            # Check for freefall condition
            self.check_freefall(magnitude, current_time)

            # Check for impact condition
            self.check_impact(magnitude)

            # Log normal readings periodically (every 10th message to reduce spam)
            if not hasattr(self, 'message_count'):
                self.message_count = 0
            self.message_count += 1

            if self.message_count % 10 == 0:
                self.get_logger().info(
                    f'Acceleration magnitude: {magnitude:.2f} m/s² '
                    f'[Normal range: 8-11 m/s²]'
                )

            # Store for next iteration
            self.last_acceleration_magnitude = magnitude

        except Exception as e:
            self.get_logger().error(f'Error in fall detection callback: {e}')


def main(args: Optional[list] = None) -> None:
    """Main entry point for the fall detector node."""
    rclpy.init(args=args)

    fall_detector = FallDetector()

    try:
        rclpy.spin(fall_detector)
    except KeyboardInterrupt:
        pass
    finally:
        fall_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

The `FallDetector` node implements a state machine that tracks the robot through potential fall phases. The `calculate_acceleration_magnitude` method uses numpy to compute the Euclidean norm of the three-axis acceleration vector - this is the total magnitude of acceleration the robot is experiencing, which should be approximately 9.81 m/s² when stationary (due to gravity).

The freefall detection logic is based on a key insight: when a robot falls freely through the air, it experiences weightlessness, similar to astronauts in orbit. The IMU measures near-zero acceleration because both the robot and the sensor are falling at the same rate. The `check_freefall` method detects when acceleration drops below 2.0 m/s² and tracks how long this condition persists. Short drops below the threshold might be sensor noise or momentary loss of contact, but sustained freefall exceeding 0.3 seconds strongly indicates a fall in progress.

The impact detection in `check_impact` watches for the opposite extreme: sudden acceleration spikes above 20.0 m/s² that occur when the robot collides with the ground. These spikes are typically 2-3 times normal gravity, creating an unmistakable signature. In a production system, detecting impact triggers emergency protocols: disabling motor torque to prevent joint damage, activating complaint control to absorb shock, or sending alerts to human supervisors.

This example demonstrates advanced IMU processing concepts: vector magnitude calculation using numpy, time-based state tracking with ROS2 Time objects, threshold-based event detection, and error handling for robustness. The fall detection patterns shown here are used in real humanoid robots to enable safer operation - they're the difference between a robot that catastrophically damages itself in a fall and one that gracefully manages the incident.

### 🤝 Practice Exercise: Trace the Balance Control Response

You're designing balance control for a humanoid robot. Given the IMU data stream below, identify when the robot is stable versus at risk of falling. For each sample, determine which balance strategy (ankle/hip/step) would be appropriate.

**Sample IMU Data to Analyze**:

| Time (ms) | Roll Angle (°) | Pitch Angle (°) | Forward Accel (m/s²) | Your Answer: Strategy Needed |
|-----------|----------------|-----------------|----------------------|------------------------------|
| 0         | 0.0            | 0.0             | 0.0                  |                              |
| 50        | 0.5            | -2.0            | -0.5                 |                              |
| 100       | 1.2            | -4.1            | -1.2                 |                              |
| 150       | 4.5            | -7.3            | -2.1                 |                              |
| 200       | 8.2            | -12.5           | -3.8                 |                              |

**Guiding Questions**:
1. At what point would ankle strategy fail? (Hint: When tilt exceeds 5 degrees)
2. When would hip strategy become necessary? (When tilt exceeds 5 degrees but is less than 15 degrees)
3. When must the robot take a step to avoid falling? (When tilt trend shows continued increase beyond hip strategy limits)
4. Why is the forward acceleration magnitude increasing as the pitch angle increases? (Consider how tilting forward causes the robot to accelerate in that direction due to gravity)

**Challenge Option** (for advanced students):
- Calculate the angular velocity (rate of change of pitch angle) between each sample
- Predict: if the current trend continues, will the robot fall? How many milliseconds until pitch exceeds 15 degrees?
- What additional sensor data would help you make a better prediction?

## Summary

**Key Takeaways**:

- **IMU Components**: Accelerometers, gyroscopes, and magnetometers work together to measure motion, rotation, and heading. Each has distinct strengths - accelerometers provide a gravity reference for tilt, gyroscopes measure precise rotation, magnetometers offer absolute heading - and weaknesses - accelerometers are noisy, gyroscopes drift over time, magnetometers suffer from magnetic interference.

- **Sensor Fusion is Essential**: No single IMU sensor is perfect for long-term orientation estimation. Gyroscope drift accumulates over hours, making long-term estimates useless. Accelerometers are noisy and affected by motion. Combining them through sensor fusion techniques creates reliable estimates, which is why the next lesson covers fusion in depth.

- **sensor_msgs/Imu Structure**: ROS2 provides a standard message format containing orientation (as a quaternion to avoid gimbal lock), angular velocity (from gyroscope), linear acceleration (from accelerometer, including gravity), and covariance matrices (indicating measurement confidence). Understanding this structure is essential for integrating IMU data into balance control systems.

- **Proprioception Enables Humanoid Control**: The combination of IMU data (body orientation and acceleration) plus joint encoders (limb positions) creates the robot's internal sense of body position and motion. This proprioceptive feedback is as critical for humanoid balance as human proprioception is for activities like walking on a narrow beam or catching yourself when you trip.

- **Balance is an Active Control Loop**: The balance system continuously cycles through sensing tilt (IMU) → computing correction (controller) → executing action (motor commands) → sensing again. With IMU sampling at 200+ Hz and total reaction times of 25-50 milliseconds, robots can prevent falls faster than humans can react.

You should now be able to explain what an IMU measures, describe the trade-offs between its three component sensors, interpret `sensor_msgs/Imu` messages in ROS2 code, and understand how IMU feedback enables humanoid robots to maintain balance and develop body awareness. You're ready to learn how combining IMU data with other sensors creates even more robust perception systems.

## Next Steps

You now understand how humanoid robots sense their own motion and orientation through inertial measurement, and how this proprioceptive feedback enables dynamic balance control in real time.

In [Lesson 4: Sensor Fusion Techniques](./04-sensor-fusion), we'll explore how to combine IMU data with cameras, depth sensors, and other perception sources to create robust, reliable perception systems. You've seen that individual sensors have critical limitations: IMUs drift over time as gyroscope errors accumulate, cameras fail in darkness or poor lighting, depth sensors struggle with reflective or transparent surfaces. Sensor fusion is the set of techniques that allow robots to overcome these individual weaknesses by intelligently combining multiple information sources.

A humanoid robot that relies solely on its IMU for balance will eventually fall as gyroscope drift accumulates and orientation estimates become meaningless. By fusing IMU data with visual odometry (tracking motion through camera frames), robots can correct drift and maintain accurate self-awareness over extended operation periods. This is the bridge from understanding individual sensors to building complete, production-ready robot perception systems that operate reliably in the real world.

This progression - from cameras to depth sensing to proprioception to sensor fusion - shows how robots build increasingly sophisticated understanding of both their world and themselves, layer by layer.
