---
title: "Lesson 1: Camera Systems and Computer Vision"
sidebar_position: 1
skills:
  - name: "Camera Types for Robotics"
    proficiency_level: "beginner"
    category: "sensor-perception"
    bloom_level: "understand"
    digcomp_area: "technical-concepts"
    measurable_at_this_level: "differentiate monocular, stereo, and RGB-D cameras for humanoid tasks"
  - name: "ROS2 Image Messages and Camera Integration"
    proficiency_level: "beginner"
    category: "robotics-middleware"
    bloom_level: "apply"
    digcomp_area: "data-processing"
    measurable_at_this_level: "subscribe to sensor_msgs/Image topics and extract camera parameters from CameraInfo"
  - name: "Camera System Design for Humanoid Robots"
    proficiency_level: "intermediate"
    category: "system-design"
    bloom_level: "analyze"
    digcomp_area: "problem-solving"
    measurable_at_this_level: "analyze camera placement trade-offs and justify selections for multi-camera architectures"
learning_objectives:
  - objective: "Understand the five foundational camera parameters (pixels, resolution, frame rate, field of view, image encoding)"
    proficiency_level: "beginner"
    bloom_level: "understand"
    assessment_method: "quiz questions 1-2, Camera Systems Review Exercise"
  - objective: "Differentiate between monocular, stereo, and RGB-D cameras based on trade-offs (cost, depth accuracy, computational load, outdoor performance)"
    proficiency_level: "beginner"
    bloom_level: "understand"
    assessment_method: "camera type comparison table, case study analysis (Atlas, Optimus)"
  - objective: "Apply ROS2 sensor_msgs/Image and CameraInfo message structures to process camera data in Python"
    proficiency_level: "beginner"
    bloom_level: "apply"
    assessment_method: "code example walkthrough, image callback implementation exercise"
  - objective: "Analyze camera placement strategies (head, wrist, chest) and their impact on humanoid robot capabilities"
    proficiency_level: "intermediate"
    bloom_level: "analyze"
    assessment_method: "multi-camera system design challenge, capstone justification section"
  - objective: "Evaluate trade-offs in camera system design for competing robot requirements (navigation, manipulation, human interaction)"
    proficiency_level: "intermediate"
    bloom_level: "evaluate"
    assessment_method: "hands-on collaboration exercise with AI assistant, design rationale documentation"
cognitive_load:
  new_concepts: 6
  assessment: "moderate - introduces five foundational camera concepts and three camera types with ROS2 integration, but builds on Module 1 ROS2 publishers/subscribers knowledge"
differentiation:
  extension_for_advanced: "Study camera calibration mathematics (Zhang's method), lens distortion correction, epipolar geometry for stereo reconstruction, and neural network-based monocular depth estimation"
  remedial_for_struggling: "Review pixel/image representation basics (RGB encoding, resolution), revisit Module 1 Lesson 1-2 (ROS2 topics, pub/sub pattern), and use visual aids to understand triangulation in stereo vision"
tags: ["ros2", "sensors", "camera", "computer-vision", "humanoid-robotics"]
generated_by: "agent"
created: "2025-12-08"
last_modified: "2025-12-08"
---

# Lesson 1: Camera Systems and Computer Vision

## What Is a Camera in Robotics?

When you watch a humanoid robot navigate a crowded room, reach for a coffee mug, or recognize a person's face, you're witnessing the power of camera systems at work. Cameras serve as the primary visual sensors that give robots the ability to perceive their environment, transforming light into digital data that algorithms can interpret. Just as human eyes capture photons and convert them into neural signals, robotic cameras capture images and convert them into pixel arrays that computer vision algorithms process.

In the context of humanoid robotics, a camera is a sensor device that captures visual information from the environment and encodes it as digital image data. This data consists of **pixels** (picture elements), each representing a tiny sample of color or brightness at a specific location in the visual field. The **resolution** of a camera—typically expressed as width × height in pixels (e.g., 1920×1080)—determines how much spatial detail the robot can perceive. The **frame rate**, measured in frames per second (fps), controls how frequently the camera captures new images, with 30 fps being common for general tasks and 60+ fps necessary for fast-moving scenarios like catching objects or maintaining balance during dynamic motion.

Two additional parameters define what a camera can see and how it delivers data. The **field of view** (FoV) specifies the angular extent of the observable world, ranging from narrow (telephoto, ~30°) to wide-angle (fisheye, 180°+). Finally, **image encoding** refers to the format in which pixel data is stored—common encodings include RGB (red-green-blue color), BGR (blue-green-red, used by OpenCV), grayscale (single intensity channel), and specialized formats like floating-point depth maps. Understanding these five foundational concepts—pixels, resolution, frame rate, field of view, and image encoding—is essential for designing and programming vision systems for humanoid robots.

## Why Camera Systems Matter for Physical AI

Camera systems are the cornerstone of physical AI because they enable robots to perform three critical categories of tasks: manipulation, navigation, and human interaction. For manipulation tasks like grasping a wrench or assembling components, cameras provide the semantic understanding necessary to identify objects, estimate their poses, and guide end-effectors with precision. Navigation systems rely on cameras to detect obstacles, recognize landmarks, and build spatial maps of environments. Human-robot interaction becomes possible when cameras enable facial recognition, gesture detection, and gaze tracking—allowing humanoids to respond naturally to people in shared spaces.

Consider three leading humanoid platforms that exemplify camera-centric design. **Boston Dynamics' Atlas** uses multiple cameras in its head and hands to perform parkour, tool manipulation, and terrain assessment—tasks that would be impossible without real-time visual feedback. **Tesla Optimus**, designed for manufacturing and domestic environments, relies exclusively on camera-based vision (no lidar) to navigate factories and recognize objects, mirroring the computer vision approach used in Tesla's autonomous vehicles. **Agility Robotics' Digit**, built for logistics and delivery, employs stereo cameras to detect stairs, avoid humans, and place packages safely—all while maintaining balance on two legs.

Without cameras, humanoid robots would be functionally blind. While other sensors like lidar provide accurate distance measurements and force sensors detect contact, cameras uniquely deliver semantic information about the world. A camera can distinguish between a coffee mug and a water bottle, recognize a stop sign, or detect whether a person is waving. This semantic richness comes at a trade-off: cameras provide less accurate depth information compared to active sensors like lidar or time-of-flight (ToF) cameras, and they struggle in low-light conditions where infrared or ultrasonic sensors excel. However, the interpretability of camera data—combined with advances in deep learning for computer vision—makes cameras indispensable for robots that must operate in human-centric environments.

The integration of cameras with modern physical AI systems creates a feedback loop: cameras capture the world, neural networks interpret the visual data, and robot control systems execute actions based on that understanding. This closed-loop vision-action pipeline is what transforms a mechanical system into an intelligent agent capable of learning from experience and adapting to novel situations.

### 📊 Case Study: Boston Dynamics Atlas - Multi-Camera Perception System

**Robot**: Atlas (Boston Dynamics)
**Camera System**: Stereo cameras in head, RGB cameras in wrists
**Application**: Autonomous parkour, tool manipulation, terrain navigation

Atlas employs a sophisticated multi-camera architecture designed for operation in unstructured environments. The head-mounted stereo camera pair (baseline approximately 10cm) provides depth perception for navigation and obstacle detection at ranges up to 8 meters, enabling Atlas to identify footholds on stairs and detect gaps during parkour maneuvers. The stereo system operates at 30 fps with 752×480 resolution per camera, balancing computational load against the need for real-time depth estimation during dynamic locomotion.

Complementing the head cameras, Atlas features wrist-mounted RGB cameras (monocular, ~1280×720) positioned to maintain visual contact with manipulation targets even when the robot's hands move outside the head camera's field of view. These eye-in-hand cameras support visual servoing for tasks like tool grasping and valve turning, where millimeter-level precision matters. The wrist cameras run at lower frame rates (10-15 fps) to reduce computational overhead, since manipulation tasks tolerate higher latency than balance control.

**Key Takeaway**: Multi-camera systems enable task specialization—head cameras prioritize depth for locomotion, while wrist cameras prioritize color detail for manipulation, each optimized for distinct computational and spatial requirements.

### 📊 Case Study: Tesla Optimus - Vision-Only Navigation

**Robot**: Optimus (Tesla)
**Camera System**: Eight monocular cameras (similar to Tesla vehicle FSD architecture)
**Application**: Factory floor navigation, bin picking, object recognition

Tesla Optimus deliberately avoids lidar and RGB-D sensors, relying exclusively on monocular cameras for all perception tasks—a design philosophy inherited from Tesla's autonomous vehicle program. The robot uses eight cameras positioned around its head and torso to achieve near-360° coverage, mirroring the camera placement in Tesla cars. Each camera captures 1280×960 resolution at 36 fps, feeding a neural network trained on Tesla's massive driving dataset to perform depth estimation, object detection, and semantic segmentation simultaneously.

The vision-only approach presents unique challenges: without active depth sensing, Optimus must infer distance from monocular cues like object size, texture gradients, and parallax motion. Tesla addresses this through large-scale data collection—thousands of hours of teleoperated robot demonstrations provide ground-truth depth labels for training neural networks to predict metric depth from single images. This strategy trades sensor cost and outdoor reliability (cameras work in sunlight, unlike IR-based depth sensors) against computational complexity and the need for extensive training data.

**Key Takeaway**: Monocular camera arrays can replace active depth sensors when combined with powerful neural network architectures and large training datasets, demonstrating that software innovation can compensate for hardware limitations in perception system design.

### 💬 AI Colearning Prompt

> **Ask your AI assistant**: "Why do stereo cameras need synchronized image capture to calculate depth accurately? What happens if the left and right camera frames are captured at slightly different times while the robot is moving?"
>
> This prompt helps you understand temporal synchronization requirements in multi-camera systems. Your AI assistant can explain how motion between unsynchronized frames causes depth estimation errors, and why hardware-triggered stereo cameras (where both sensors capture simultaneously) produce more accurate depth maps than software-synchronized systems, especially critical for fast-moving humanoid robots.

## Key Principles

### 3.1 Camera Types and Trade-offs

Humanoid robots employ three primary camera types, each with distinct advantages and limitations shaped by the physics of depth perception.

**Monocular cameras** use a single lens to capture 2D images, similar to taking a photograph with your phone. Their primary advantage is simplicity: they're lightweight, inexpensive, and require minimal computational overhead. Tesla Optimus uses monocular cameras for object detection and scene understanding, relying on deep learning models to infer depth from visual cues like texture gradients and object size. However, monocular cameras inherently lack depth information—they cannot directly measure how far away objects are without additional processing or motion (e.g., structure from motion). This makes them suitable for classification tasks (identifying objects) but challenging for precise manipulation or obstacle avoidance.

**Stereo cameras** mimic human binocular vision by using two synchronized cameras separated by a baseline distance (typically 6-12 cm for humanoid heads). By comparing the left and right images and identifying corresponding pixels, stereo algorithms compute depth through triangulation. The depth accuracy depends on the baseline: wider baselines provide better depth precision at long ranges but lose accuracy for nearby objects. Stereo cameras work passively (no active illumination required) and are effective outdoors, making them popular for navigation tasks. Boston Dynamics' Atlas uses stereo vision for terrain mapping during locomotion. The primary disadvantage is computational cost: stereo matching algorithms must solve the correspondence problem for thousands of pixels in real-time, and performance degrades in textureless environments (e.g., blank walls).

**RGB-D cameras** (RGB plus depth) actively measure distance using either structured light projection (e.g., Microsoft Kinect) or time-of-flight (ToF) technology (e.g., Intel RealSense D435). These cameras output both color images and per-pixel depth maps without requiring correspondence solving. Structured light systems project infrared patterns and measure distortion, while ToF cameras emit infrared pulses and measure round-trip time. RGB-D cameras excel at indoor grasping tasks because they provide immediate, dense depth information for object pose estimation. Their limitations include restricted outdoor performance (sunlight interferes with infrared), limited range (typically 0.3-10 meters), and higher power consumption compared to passive cameras.

Here's a comparison of the three camera types:

| Feature | Monocular | Stereo | RGB-D |
|---------|-----------|--------|-------|
| Depth Info | None (2D) | Triangulation | Active (IR/ToF) |
| Range | N/A | 0.5-10m | 0.3-10m |
| Advantages | Simple, low cost | Passive depth | No computation for depth |
| Disadvantages | No depth | Requires calibration | Limited outdoor use |
| Humanoid Use | Object detection | Navigation | Indoor grasping |

### 3.2 Camera Parameters and Their Effects

Selecting camera parameters requires balancing competing priorities: resolution, field of view, and frame rate each impose trade-offs on computational resources, weight, and performance.

**Resolution** determines spatial detail. Common resolutions range from VGA (640×480) for low-latency applications to 4K (3840×2160) for inspection tasks. Higher resolution enables finer object detection (e.g., reading small text or detecting distant objects) but quadratically increases data volume and processing time. For humanoid robots, 640×480 or 1280×720 represents a practical sweet spot: sufficient detail for navigation and manipulation without overwhelming onboard computers. Tesla Optimus reportedly uses 1280×960 cameras to balance detail and inference speed.

**Field of view** defines the angular coverage of the camera, typically expressed in degrees horizontally. Narrow fields of view (30-50°, telephoto lenses) provide magnified detail at distance but sacrifice peripheral awareness. Wide fields of view (90-120°, wide-angle lenses) capture more context but distort edges and reduce central detail. The relationship between focal length and FoV is inverse: shorter focal lengths yield wider views. For humanoid head-mounted cameras, 60-90° horizontal FoV is common, matching human peripheral vision. Manipulation cameras on wrists often use narrower 40-60° FoV to focus on the workspace directly ahead.

**Frame rate** controls temporal resolution. Standard rates include 30 fps (sufficient for walking and general navigation), 60 fps (enables smooth tracking of moving objects), and 120+ fps (required for high-speed manipulation or catching). Higher frame rates increase data bandwidth linearly and reduce exposure time (potentially requiring more lighting). For balance-critical tasks where visual feedback informs control loops, 60 fps or higher minimizes latency between perception and action.

### 3.3 Camera Placement on Humanoid Robots

Where you mount cameras fundamentally shapes what tasks a humanoid can perform. Three canonical locations each serve distinct purposes.

**Head-mounted cameras** (typically stereo pairs or RGB-D) provide the robot's primary situational awareness, analogous to human eyes. Positioned 1.5-1.8 meters above ground on adult-scale humanoids, they offer a commanding view for navigation, person detection, and scene understanding. The head's ability to pan and tilt (via neck actuators) extends the effective field of view and allows the robot to track moving targets or inspect specific regions. The primary disadvantage is that head cameras cannot see the robot's own hands during manipulation tasks unless the head continuously tracks the hands—creating a coordination overhead.

**Wrist-mounted cameras** (often monocular or small RGB-D sensors) solve the manipulation visibility problem by maintaining a constant view of the end-effector workspace. These "eye-in-hand" cameras enable visual servoing (adjusting grasp approach based on real-time feedback) and verification (confirming object contact). Boston Dynamics' Atlas uses wrist cameras to guide tool grasping. The limitation is that wrist cameras provide a restricted, task-focused view and do not contribute to global navigation or scene awareness.

**Chest-mounted cameras** represent a compromise: positioned at torso height with a fixed forward orientation, they maintain a stable viewpoint unaffected by head motion while still capturing the hands during reaching tasks. Some humanoid designs use chest cameras for odometry (tracking motion by monitoring floor features) or as a fallback navigation sensor. However, chest cameras sacrifice the flexibility of head-mounted systems and may be occluded by the robot's own arms.

Many modern humanoids employ all three camera placements simultaneously, fusing data across sensors. This multi-camera strategy mirrors human vision-motor coordination: your eyes (head cameras) survey the scene, your peripheral vision (chest cameras) monitors obstacles, and you glance at your hands (wrist cameras) when precision matters.

### 3.4 Image Data Representation in ROS2

In ROS2, camera data flows through the system as `sensor_msgs/Image` messages—a standardized format that separates image metadata from raw pixel data. Understanding this message structure is essential for subscribing to camera topics and integrating vision algorithms.

The `sensor_msgs/Image` message contains seven fields:

- **header**: A standard ROS2 header including a timestamp (when the image was captured) and frame_id (the coordinate frame of the camera, e.g., "camera_optical_frame"). Timestamps enable synchronization across multiple sensors.
- **height** and **width**: Integer values specifying image dimensions in pixels.
- **encoding**: A string describing the pixel format, such as "rgb8" (8-bit RGB), "bgr8" (8-bit BGR), "mono8" (grayscale), or "32FC1" (32-bit floating-point depth map). This field determines how to interpret the raw data array.
- **is_bigendian**: A boolean indicating byte order (rarely needed on modern little-endian systems).
- **step**: The number of bytes per row, accounting for potential padding. For an 8-bit RGB image of width W, step = W × 3 (three bytes per pixel).
- **data**: A flat byte array containing the actual pixel values in row-major order (left-to-right, top-to-bottom).

**Color space encodings** warrant special attention because different libraries use different conventions. "rgb8" places red in the first byte, green in the second, blue in the third. "bgr8" reverses this order—a quirk inherited from OpenCV, which defaults to BGR. Grayscale encodings like "mono8" use a single byte per pixel. Floating-point encodings like "32FC1" store depth values in meters, common for RGB-D cameras.

**Memory layout** follows row-major order: pixel (0,0) is the top-left corner, and pixels are stored row-by-row. To access the pixel at row `r`, column `c` in an "rgb8" image, compute the byte offset as `offset = r * step + c * 3`, then read three consecutive bytes for red, green, and blue channels.

This standardized representation allows ROS2 nodes written in different languages (Python, C++) to exchange image data seamlessly. When you subscribe to a camera topic, you receive these `sensor_msgs/Image` messages at the configured frame rate, ready for processing.

### 3.5 Image Flow Through ROS2 Nodes

Camera data in ROS2 follows a publisher-subscriber pattern that decouples image acquisition from processing. A **camera driver node** (e.g., `usb_cam_node` for USB cameras or vendor-specific drivers for industrial cameras) publishes `sensor_msgs/Image` messages on a topic like `/camera/image_raw`. Downstream nodes—your vision algorithms, display tools, or recording utilities—subscribe to this topic and receive each new frame via callback functions.

Accompanying the image topic, most camera drivers publish a **sensor_msgs/CameraInfo** message on `/camera/camera_info`. This message contains intrinsic camera parameters (focal length, principal point, distortion coefficients) necessary for tasks like undistorting images or projecting 3D points into the image plane. Stereo algorithms require synchronized CameraInfo from both left and right cameras.

**Quality of Service (QoS) settings** govern message delivery reliability. For camera topics, common practice uses "best effort" reliability (tolerates dropped frames to minimize latency) with a moderate queue depth (e.g., 10 messages). This configuration ensures real-time performance: if processing lags behind the camera frame rate, old frames are discarded rather than queuing indefinitely. For critical applications like recording, you might use "reliable" delivery at the cost of increased latency.

The image flow pipeline typically looks like this:

1. Camera driver captures frame from hardware
2. Driver publishes `sensor_msgs/Image` on `/camera/image_raw`
3. Vision node receives message via callback (e.g., `image_callback`)
4. Node processes image (e.g., object detection with OpenCV/PyTorch)
5. Node publishes results (e.g., bounding boxes, detected classes) on output topics

This modular design allows you to swap camera hardware (changing only the driver node) or upgrade vision algorithms (changing only the processing node) without rewriting the entire system.

### 🎓 Expert Insight: RGB vs BGR Color Space Confusion

> **Common Pitfall**: One of the most frequent errors in ROS2 vision pipelines is forgetting that OpenCV uses BGR (Blue-Green-Red) color channel ordering by default, while most other libraries and ROS2 Image messages use RGB (Red-Green-Blue). When you subscribe to an "rgb8" encoded image and pass it directly to OpenCV functions like `cv2.imshow()`, colors will appear inverted (reds become blues, blues become reds).
>
> **Best practice**: Always check the encoding field in your image callback. If working with OpenCV, either convert RGB to BGR using `cv2.cvtColor(img, cv2.COLOR_RGB2BGR)` or explicitly request "bgr8" encoding when creating your camera driver. This simple check prevents hours of debugging "why does my red ball appear blue?"

### 💬 AI Colearning Prompt

> **Ask your AI assistant**: "Explain the relationship between field of view and focal length using a binoculars analogy. Why do wide-angle lenses have shorter focal lengths?"
>
> Understanding the inverse relationship between focal length and field of view is crucial for selecting the right lens for a task. Your AI assistant can clarify this optical principle by comparing camera lenses to adjustable binoculars: zooming in (increasing focal length) narrows your view but magnifies distant objects, while zooming out (decreasing focal length) widens your view but reduces magnification. This analogy helps build intuition for why humanoid robots use wide-angle lenses for navigation (maximum awareness) and telephoto lenses for inspection tasks (detailed examination at distance).

## Embedded Callouts

### 🎓 Common Pitfall: Resolution Isn't Everything

> **Resolution sweet spot for humanoids**: Beginners often assume higher resolution always improves performance. In reality, 640×480 resolution is often superior to 4K for real-time humanoid applications. Why? Quadrupling resolution (e.g., 640×480 to 1280×960) quadruples pixel count, data transfer bandwidth, and processing time, but rarely quadruples task performance. For navigation and manipulation, the limiting factor is usually algorithm latency, not spatial detail. Modern object detectors (YOLO, Faster R-CNN) are trained on ~640-pixel images and perform well at this scale. Only specialized tasks—reading small text, inspecting fine defects, or detecting distant objects—justify resolutions above 1280×720 on resource-constrained robot hardware.

### 🤝 Hands-On Collaboration Exercise: Multi-Camera System Design

> **Design challenge for you and your AI assistant**: Imagine you're designing the vision system for a humanoid delivery robot that must navigate office buildings, ride elevators, and hand packages to people. The robot has three available camera mounts (head, chest, wrists) and a computational budget for processing three camera streams.
>
> Work with your AI assistant to decide: Which camera types (monocular, stereo, RGB-D) would you place at each location, and why? Consider these competing requirements: (1) safe navigation requires wide FoV and depth sensing, (2) object recognition (detecting package labels) requires color and resolution, (3) human interaction requires face detection and gaze estimation, and (4) hand-off tasks require precise depth at close range.
>
> There's no single "correct" answer—the goal is to articulate trade-offs and defend your choices. Your AI assistant can role-play as a hardware engineer, challenging your decisions with budget and power constraints. This exercise mirrors real-world system design discussions on humanoid robotics teams.

## Practical Example: Subscribing to Camera Images

The following Python code demonstrates the fundamental pattern for receiving camera images in a ROS2 node. This subscriber node connects to a camera driver's output topic and logs metadata about each received frame.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraSubscriber(Node):
    """Subscribes to camera images and logs image metadata."""

    def __init__(self) -> None:
        super().__init__('camera_subscriber')
        # Subscribe to the raw image topic (typical ROS2 camera driver output).
        # Image messages arrive asynchronously; ROS2 queues them in order.
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10  # QoS queue depth: keep latest 10 messages; drop older ones
        )

    def image_callback(self, msg: Image) -> None:
        """Process incoming camera image and log metadata.

        This callback is invoked by the ROS2 executor each time a new frame
        arrives on '/camera/image_raw'. The msg parameter is a sensor_msgs/Image
        object containing all metadata and pixel data for a single frame.
        """
        # Access image dimensions and format from ROS2 message fields
        self.get_logger().info(
            f'Image received: {msg.width}x{msg.height}, '
            f'encoding={msg.encoding}, timestamp={msg.header.stamp.sec}'
        )
        # Image processing (e.g., OpenCV operations) would go here.
        # Example: convert to NumPy with cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # or manually: np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)

# Note: rclpy.init(), spin(), shutdown() omitted for conceptual clarity
```

The `image_callback` method executes every time a new frame arrives on the `/camera/image_raw` topic. Inside this callback, you have access to the complete `sensor_msgs/Image` message, including dimensions (`msg.width`, `msg.height`), pixel format (`msg.encoding`), timestamp (`msg.header.stamp`), and raw pixel data (`msg.data`). This example logs metadata to verify the subscription is working—a crucial debugging step when integrating new cameras.

The `sensor_msgs/Image` message contains everything needed to reconstruct the image: if the encoding is "bgr8" (common for USB cameras), you can convert `msg.data` to a NumPy array and pass it to OpenCV functions like `cv2.imshow()` for display or `cv2.Canny()` for edge detection. For humanoid vision pipelines, the callback typically performs feature extraction (e.g., detecting faces with a neural network), publishes results to downstream nodes (e.g., control systems), or records images for offline analysis.

### Accessing Camera Calibration Parameters with CameraInfo

Alongside the image topic, ROS2 camera drivers publish **sensor_msgs/CameraInfo** messages containing intrinsic camera parameters—focal length, principal point, and distortion coefficients. These parameters are essential for geometric tasks: undistorting images, projecting 3D world points into the image plane, or estimating object poses from image coordinates. The following example demonstrates subscribing to camera calibration data and extracting key parameters like focal length and field of view.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import math

class CameraInfoSubscriber(Node):
    """Subscribes to camera calibration info and extracts optical parameters."""

    def __init__(self) -> None:
        super().__init__('camera_info_subscriber')
        # Subscribe to the camera_info topic published alongside image_raw.
        # CameraInfo messages are typically published at lower frequency (1-10 Hz)
        # since calibration parameters remain constant during operation.
        self.subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.info_callback,
            10
        )

    def info_callback(self, msg: CameraInfo) -> None:
        """Extract camera parameters from intrinsic calibration matrix.

        The K matrix (intrinsic matrix) is a 3x3 array flattened to 9 elements:
        [fx,  0, cx,
         0,  fy, cy,
         0,   0,  1]
        where fx/fy are focal lengths (pixels) and cx/cy define the principal point.
        """
        # Extract intrinsic parameters from the K matrix (row-major storage)
        fx: float = msg.k[0]  # Focal length in x (pixels)
        fy: float = msg.k[4]  # Focal length in y (pixels)
        cx: float = msg.k[2]  # Principal point x (image center, pixels)
        cy: float = msg.k[5]  # Principal point y (image center, pixels)

        # Calculate horizontal field of view from focal length and image width.
        # This accounts for actual image sensor dimensions, not just resolution.
        fov_x_rad: float = 2.0 * math.atan(msg.width / (2.0 * fx))
        fov_x_deg: float = fov_x_rad * 180.0 / math.pi

        # Log calibration parameters for verification during integration
        self.get_logger().info(
            f'Camera calibration: {msg.width}x{msg.height}, '
            f'focal_length_x={fx:.1f}px, FOV={fov_x_deg:.1f}°, '
            f'principal_point=({cx:.1f}, {cy:.1f})'
        )
```

This CameraInfo subscriber demonstrates how to extract the intrinsic camera matrix (K matrix) and compute the field of view—essential for tasks like rectifying distorted images or determining if a detected object at a specific pixel location falls within the manipulator's reachable workspace. The K matrix is the foundation of camera projection: multiplying a 3D world point by K yields the pixel location where that point appears in the image, enabling geometric reasoning about objects in the scene.

This subscriber pattern is foundational: whether you're building a simple image logger or a sophisticated multi-stage vision pipeline with object tracking, pose estimation, and scene reconstruction, every ROS2 vision node begins by subscribing to camera topics and implementing robust callbacks that process image data in real-time.

### 🤝 Practice Exercise: Inspecting Live Camera Topics

> **Challenge**: If you have access to a ROS2 system with a camera (physical hardware, Gazebo simulation, or rosbag playback), use command-line tools to inspect camera topics:
>
> 1. List all active topics: `ros2 topic list`
> 2. Find the camera image topic (usually `/camera/image_raw` or similar)
> 3. Display the message type: `ros2 topic info /camera/image_raw`
> 4. Echo one message to see its structure: `ros2 topic echo /camera/image_raw --once`
> 5. Check the publishing rate: `ros2 topic hz /camera/image_raw`
> 6. Inspect CameraInfo: `ros2 topic echo /camera/camera_info --once`
>
> **Goal**: Extract the image width, height, encoding, and camera's horizontal field of view (calculate from the K matrix focal length). Compare the actual frame rate to the camera's specification.
>
> **Hint**: For step 6, look for the `k` array in the output. The first element `k[0]` is the focal length in pixels. Use the FOV formula from the CameraInfoSubscriber code example.

## Summary

- **Cameras transform light into digital pixel arrays**, providing humanoid robots with semantic understanding of their environment through five key parameters: pixels, resolution, frame rate, field of view, and image encoding.
- **Three camera types serve different roles**: monocular cameras offer simplicity for 2D tasks, stereo cameras provide passive depth through triangulation for navigation, and RGB-D cameras deliver active depth sensing for precise indoor manipulation.
- **Camera placement determines capabilities**: head-mounted cameras enable situational awareness and navigation, wrist-mounted cameras support manipulation tasks with visual servoing, and chest-mounted cameras offer a stable intermediate viewpoint.
- **ROS2 standardizes image data as sensor_msgs/Image messages** with separate fields for dimensions, encoding (rgb8, bgr8, mono8, etc.), timestamps, and raw pixel data stored in row-major order—enabling language-agnostic image exchange across nodes.
- **The publisher-subscriber pattern decouples image acquisition from processing**, allowing camera driver nodes to publish frames on topics like `/camera/image_raw` while vision algorithms subscribe and process images via callbacks, using "best effort" QoS for real-time performance.

## Next Steps

Now that you understand how cameras capture and represent visual information in ROS2, the natural next question is: how do we accurately measure distance to objects in 3D space? While monocular cameras lack depth and stereo cameras require computational correspondence solving, dedicated depth sensing technologies offer direct distance measurements critical for navigation and manipulation.

In Lesson 2, we'll explore depth sensing technologies—including time-of-flight sensors, structured light systems, and lidar—that complement camera systems by providing millimeter-accurate range data. You'll learn how to fuse depth information with camera images to create rich 3D representations of the robot's workspace, enabling tasks like obstacle avoidance, grasp planning, and terrain mapping.

Continue to [Lesson 2: Depth Sensing Technologies](./02-depth-sensing) to build on your camera fundamentals and unlock 3D perception capabilities.
