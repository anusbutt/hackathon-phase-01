# Module 2 Capstone Project: Integrated Sensor System Design

## Introduction

Professional roboticists design sensor systems through iterative refinement, balancing performance, cost, reliability, and computational constraints. This capstone challenges you to design a complete perception system for a humanoid robot, integrating camera systems (Lesson 1), depth sensing (Lesson 2), IMU and proprioception (Lesson 3), and sensor fusion (Lesson 4).

### Why a Design Document?

Professional robotics begins with architecture and justification before implementation. Design documents force explicit decision-making about trade-offs—you can't hide behind "I'll try different sensors and see what works." This mirrors industry practice: technical design reviews, RFP responses, and architectural documentation all precede development.

Individual lessons covered sensors in isolation, but real humanoids require coordinated multi-sensor systems. Trade-offs become explicit: camera resolution vs. compute budget, LiDAR range vs. cost, IMU drift vs. fusion complexity. Your design demonstrates how fusion strategies address sensor limitations from Lessons 1-4.

This capstone develops design documentation skills critical for robotics engineers. Whether writing grant proposals, presenting technical reviews, or collaborating with teams, justifying decisions with evidence—not arbitrary choices—distinguishes professional engineering.

### Learning Objectives

By completing this capstone, you will:

- **Apply** camera, depth, IMU, and fusion concepts to realistic humanoid robotics challenges
- **Justify** sensor selection with technical trade-offs (resolution vs. latency, range vs. accuracy, cost vs. redundancy)
- **Design** ROS2 architectures with appropriate message types and data flow
- **Analyze** failure modes and propose graceful degradation strategies
- **Communicate** technical decisions through structured documentation

---

## Capstone Scenario Options

Select **one** scenario for your design. Each requires applying all Module 2 concepts.

### Scenario A: Indoor Home Assistant Robot

**Scenario**: Design a humanoid robot for household tasks in urban apartments. Navigate autonomously through varying lighting (bright kitchen, dim bedroom), avoid furniture/pets/humans, identify and manipulate objects (pick up books, retrieve mugs from shelves, place items in drawers), and maintain balance on different surfaces (tile, carpet, rugs) while carrying 5 kg loads.

**Constraints**: Limited compute (embedded GPU or CPU-only), glass surfaces and mirrors, $5,000 sensor budget.

**Camera Systems (Lesson 1)**: Object recognition for household items, visual servoing for grasping precision, landmark-based localization. **Challenge**: Lighting variability from sunlight to lamp lighting affects exposure and detection.

**Depth Sensing (Lesson 2)**: Obstacle avoidance (0.5-5m), grasp distance measurement (0.3-2m), surface detection for floors/tables/shelves. **Challenge**: IR-based sensors (structured light, ToF) fail on glass/mirrors—requires stereo vision or fusion strategies.

**IMU (Lesson 3)**: Balance control detecting tilt while walking/carrying, fall detection for emergency stops, motion prediction for stable gait. **Challenge**: Drift accumulation after 10+ minutes requires re-initialization or fusion.

**Fusion (Lesson 4)**: Visual-Inertial Odometry for GPS-free indoor localization, depth+IMU fusion for gait adjustment, redundancy enabling navigation with camera failure. **Challenge**: Synchronizing different rates (camera 15 Hz, IMU 100 Hz, depth 30 Hz) using `message_filters`.

**Success**: Sensor suite enables detection, distance measurement, balance, localization within $5,000. VIO fusion for drift-free navigation. Address camera glare, depth failures on glass, IMU drift. ROS2 uses `sensor_msgs/Image`, `PointCloud2`, `Imu`.

---

### Scenario B: Outdoor Delivery Robot

**Scenario**: Design a humanoid for package delivery on university campuses and office parks. Walk on uneven terrain (grass, gravel, curbs, stairs), avoid dynamic obstacles (pedestrians, bicyclists, vehicles), navigate 2 km trips using GPS-aided localization, operate in variable weather (sunny, cloudy, light rain) during daylight, and carry 15 kg packages while maintaining stability.

**Constraints**: Outdoor lighting (direct sunlight, shadows), long-range detection (5-30m), battery-limited compute, $10,000 budget.

**Camera Systems (Lesson 1)**: Semantic navigation (sidewalks, crosswalks, building entrances), visual landmark recognition, texture analysis for curb detection. **Challenge**: Direct sunlight causes overexposure; shadows create high contrast requiring HDR or exposure bracketing.

**Depth Sensing (Lesson 2)**: Long-range obstacle detection (5-30m) for reaction time, terrain mapping for footstep planning, curb/stair detection for gait adaptation. **Challenge**: Sunlight interferes with IR-based sensors—requires LiDAR for outdoor reliability (higher cost/power).

**IMU (Lesson 3)**: Tilt detection on slopes for posture adjustment, vibration damping from uneven surfaces, package load sensing for center-of-mass shifts. **Challenge**: Magnetic interference near buildings affects magnetometer calibration.

**Fusion (Lesson 4)**: LiDAR+IMU SLAM for outdoor localization, GPS (10m accuracy) + visual odometry (cm-level relative) for global+local precision, multi-modal obstacle fusion merging LiDAR and stereo depth. **Challenge**: GPS dropout in tree cover requires seamless VIO fallback.

**Success**: Handle outdoor lighting, long-range obstacles, uneven terrain, loads within $10,000. Justify LiDAR over RGB-D using sunlight limitations from Lesson 2. Fusion uses LiDAR+IMU SLAM with GPS fallback. Address GPS dropout, sun glare, magnetic interference. ROS2 includes `LaserScan` or `PointCloud2`, `NavSatFix`, `Imu`.

---

### Scenario C: Human Interaction Robot (Healthcare)

**Scenario**: Design a humanoid for healthcare settings (hospitals, nursing homes). Approach humans safely without startling (0.5-1.5m social distance), track faces/gestures for non-verbal communication and eye contact, hand off objects (medication, water bottles) safely, maintain stable head orientation for natural gaze, and detect human distress (falls, sudden movements) to alert caregivers.

**Constraints**: Human safety paramount (zero collision tolerance), privacy (minimal data storage), indoor only, $7,500 budget.

**Camera Systems (Lesson 1)**: Face detection/tracking for gaze direction, gesture recognition (waving, pointing, reaching), visual servoing for object handoff. **Challenge**: Privacy requires edge-only processing without cloud storage, impacting compute and algorithm selection.

**Depth Sensing (Lesson 2)**: Safe approach distance measurement (0.5-3m) for personal space, 3D person tracking combining RGB face detection with depth, obstacle detection for medical equipment (IV poles, wheelchairs). **Challenge**: Wheelchair users/children require lower sensor placement—chest-mounted depth may miss low obstacles.

**IMU (Lesson 3)**: Stable head orientation keeping head level during walking for natural gaze, gentle motion control with smooth acceleration to avoid startling, collision reaction detecting unexpected contact via accelerometer spikes. **Challenge**: Distinguish intentional touch (patting) from collisions.

**Fusion (Lesson 4)**: Camera+depth 3D person tracking, IMU+vision head stabilization using IMU to predict head motion and compensate camera tracking, multi-sensor safety layer with redundant human detection. **Challenge**: Synchronize camera face detection (15 Hz) with depth (30 Hz) using `message_filters`.

**Success**: Enable face tracking, safe distance, stable head, gesture recognition within $7,500. Fusion uses camera+depth 3D tracking with IMU head stabilization. Address privacy via edge processing and no persistent storage. Failure modes: camera occlusion, depth close-range limits, redundant human detection. ROS2 shows `ApproximateTimeSynchronizer` for camera+depth sync.

---

### Scenario D: Warehouse Logistics Robot

**Scenario**: Design a humanoid for high-speed warehouse logistics (Amazon fulfillment centers). Navigate aisles at 3 m/s avoiding forklifts/workers/robots, identify shelf locations using barcode/QR scanning, pick items from floor to 2.5m overhead, track rapid acceleration during start/stop cycles for balance, and operate in structured environments with known floor plans but dynamic obstacles.

**Constraints**: High-speed requires low latency (&lt;50 ms), precise localization (&lt;5 cm error), harsh fluorescent lighting, $12,000 budget.

**Camera Systems (Lesson 1)**: High-resolution (1080p+) barcode/QR scanning at 0.5-2m, multi-camera visual odometry for precise localization, shelf edge detection for reaching. **Challenge**: Overhead fluorescent flicker (60 Hz) affects exposure—global shutter cameras may be needed for fast motion.

**Depth Sensing (Lesson 2)**: High-speed obstacle detection via LiDAR with &lt;50 ms latency for safe 3 m/s navigation, precise shelf localization via 3D mapping (±2 cm accuracy), item dimension measurement. **Challenge**: Fast motion causes depth camera motion blur—LiDAR less affected.

**IMU (Lesson 3)**: Rapid acceleration tracking (200+ Hz) for sudden starts/stops, lateral acceleration detection for high-speed turns and dynamic stability, vibration isolation filtering concrete floor vibration. **Challenge**: High update rate (200 Hz) generates significant data throughput (bandwidth constraint).

**Fusion (Lesson 4)**: Multi-camera visual odometry fusing 3+ cameras for redundant drift-free localization, LiDAR+Visual SLAM combining structure with features (LOAM, LIO-SAM), IMU-aided camera stabilization using IMU prediction to compensate motion blur. **Challenge**: Fuse high-rate IMU (200 Hz) with lower-rate cameras (30 Hz) via predictive filtering.

**Success**: Enable high-speed navigation, barcode scanning, precise shelf localization, rapid acceleration tracking within $12,000. Fusion uses multi-camera visual odometry with LiDAR and IMU. Latency analysis shows &lt;50 ms obstacle detection pipeline. Failure modes: camera motion blur, LiDAR occlusion by forklifts, IMU saturation during emergency stops. ROS2 addresses data throughput via QoS settings (best-effort for cameras, reliable for critical control), compression, prioritization.

---

## Design Requirements

Your design document must address the following:

### Sensor Selection and Justification

**Camera System**: Specify type (monocular RGB, stereo, RGB-D, combination), resolution (640×480 to 1920×1080), frame rate (15-60 FPS), field of view (60-120°). Example models: Intel RealSense D435 (RGB-D 640×480), ZED 2 (stereo 1080p), generic USB webcam. **Justification must reference Lesson 1 trade-offs**. Example: "Selected 640×480 over 1080p because navigation requires only basic shape recognition, and lower resolution reduces latency from 150ms to 50ms on embedded GPU."

**Depth Sensor**: Specify technology (LiDAR mechanical/solid-state, structured light RGB-D, ToF), range (0.5-30m), accuracy (±2 to ±10 cm), update rate (10-100 Hz). Examples: Velodyne Puck (mechanical LiDAR, 100m), Intel RealSense L515 (solid-state LiDAR, 9m), Azure Kinect (ToF, 5.5m). **Justify using Lesson 2 sensor limitations**. Example: "Selected LiDAR over RGB-D for outdoor scenario because structured light/ToF fail in sunlight due to IR interference (Lesson 2). Though LiDAR costs $8,000 vs. $400 for RGB-D, outdoor reliability justifies expense."

**IMU**: Specify update rate (50-200 Hz), accelerometer range (±2g to ±16g), gyroscope range (±250°/s to ±2000°/s). Examples: MPU-9250 (9-axis, 100 Hz, $15), VectorNav VN-100 (200 Hz, $2,000), Xsens MTi ($1,500). **Justify using Lesson 3 trade-offs**. Example: "Selected 200 Hz IMU for warehouse to track rapid acceleration during emergency stops at 3 m/s. Lower 50 Hz would miss acceleration peaks &lt;20ms."

### Sensor Placement and Coverage

Diagram sensor placement: **Head-mounted** cameras for navigation/face tracking with pan/tilt; **chest-mounted** LiDAR/depth for stable SLAM with minimal arm occlusion; **wrist-mounted** cameras for manipulation visual servoing; **torso IMU** for whole-body balance. Show FOV cones, identify coverage gaps (e.g., "blind spot behind robot, mitigated by 180° rotation before backing"), demonstrate redundancy (overlapping camera+LiDAR FOV forward). Hand-drawn diagrams acceptable if legible.

### ROS2 Integration Architecture

Specify node structure: **Sensor drivers** (`camera_driver_node`, `lidar_driver_node`, `imu_driver_node`) publish raw data. **Processing nodes** (`object_detector_node`, `slam_node`, `balance_controller_node`) subscribe and publish processed outputs. **Fusion node** (`sensor_fusion_node`) subscribes to multiple sensors, publishes fused state estimates.

**Topics and message types**: Camera `/camera/image_raw` (`sensor_msgs/Image`), `/camera/camera_info` (`sensor_msgs/CameraInfo`); depth `/depth/points` (`sensor_msgs/PointCloud2`) or `/scan` (`sensor_msgs/LaserScan`); IMU `/imu/data` (`sensor_msgs/Imu`); fused `/odometry/filtered` (`nav_msgs/Odometry`) from `robot_localization`.

**QoS policies**: Cameras use best-effort for real-time priority; queue depth 5-10 for high-rate sensors, 1-2 for control commands; critical safety uses reliable QoS.

Include diagram (hand-drawn or digital) showing nodes, topics, message types with data flow arrows.

### Sensor Fusion Strategy

**Algorithm selection**: Complementary Filter (IMU + low-rate orientation, simple/low-compute), Extended Kalman Filter (camera + IMU VIO, nonlinear motion), `robot_localization` (pre-built EKF for IMU/odometry/GPS), VIO algorithms (ORB-SLAM, VINS-Mono for camera+IMU), LiDAR+IMU SLAM (LOAM, LIO-SAM for outdoor 3D mapping).

**State variables**: Position (x, y, z), orientation (roll, pitch, yaw), linear velocity (vx, vy, vz), angular velocity (ωx, ωy, ωz).

**Sensor contributions**: Example: "Camera provides position via visual odometry at 15 Hz; IMU provides orientation and angular velocity at 100 Hz; depth provides obstacle constraints preventing position drift into walls; fusion uses EKF with IMU prediction and camera correction."

Include fusion block diagram: sensor inputs → fusion algorithm (with rates) → state estimate → consuming nodes.

### Failure Modes and Graceful Degradation

Analyze 3-5 scenarios:

**Camera Failure** (lens obscured, sun glare): Detection via no new messages >1 sec or >95% pixels saturated. Degradation: fall back to depth+IMU for obstacle avoidance and dead reckoning, disable object recognition. Impact: cannot identify objects/landmarks, navigation limited to mapped areas.

**Depth Sensor Failure** (glass false readings, hardware failure): Detection via all NaN values in PointCloud2 or implausible ranges (&lt;0.1m away from walls). Degradation: use stereo vision for depth (if available) or rely on IMU+known map. Impact: reduced obstacle detection range, cannot safely approach unknown objects.

**IMU Drift** (bias accumulation >10 min): Detection via orientation divergence from visual horizon or gravity by &gt;5°. Degradation: re-initialize IMU with camera-based horizon or magnetometer. Impact: temporary disorientation 1-2 sec during re-init, reduced balance accuracy.

**Sensor Sync Failure** (camera and depth timestamps differ >100 ms): Detection via `ApproximateTimeSynchronizer` failure to match messages. Degradation: use sensors independently or increase slop from 50ms to 100ms. Impact: reduced fusion accuracy, potential camera-depth mismatch causing incorrect 3D positions.

**Complete Suite Failure** (safety-critical): Detection via all sensor topics silent >2 sec. Degradation: emergency stop, audible alarm, await manual intervention. Impact: cannot operate autonomously, requires human recovery.

Present as table: Failure Mode | Detection Method | Degradation Strategy | Performance Impact.

### Performance Criteria

**Latency**: Define sensor-to-decision budget. Example: "Obstacle detection to path replanning &lt;200 ms: camera capture 30 ms + network 10 ms + processing 100 ms + planning 60 ms."

**Accuracy**: Set localization tolerances. Example: "±5 cm for warehouse shelf alignment, ±50 cm for outdoor waypoints. Depth ±2 cm at 1m for grasping."

**Update Rates**: Minimum sensor rates. Example: "Camera 15 Hz for object recognition, 30 Hz for visual odometry; IMU 100 Hz for balance; depth 30 Hz for obstacles."

**Compute Cost**: Estimate load. Example: "Stereo depth 30% GPU, object detection 40% GPU, fusion 10% CPU = 80% total, 20% margin. If exceeded, reduce camera resolution 1080p→720p, saving 15% GPU."

**Cost Budget**: Break down total. Example: "LiDAR $8,000 + cameras $1,500 + IMU $500 = $10,000 total, meeting outdoor scenario budget."

---

## Deliverables

Submit a design document (PDF or Markdown, 2-3 pages excluding diagrams) with:

**1. Scenario Selection** (0.25 pages): Which scenario (A/B/C/D), why it interests you, brief overview of robot's primary task.

**2. Sensor Specifications** (0.5 pages): Table with Sensor Type | Model Example | Specifications | Placement | Update Rate | Cost. Justification paragraphs for each sensor referencing Lessons 1-4 trade-offs.

**3. ROS2 Node Architecture** (0.5 pages): Visual diagram (hand-drawn or digital) with node/topic/message type labels. 1-2 paragraphs describing data flow from drivers through processing to fusion.

**4. Sensor Fusion Strategy** (0.5 pages): Algorithm selection with rationale. Block diagram showing sensor inputs → fusion → state outputs. 2-3 paragraphs explaining which sensors contribute to which state variables, referencing Lesson 4.

**5. Failure Mode Analysis** (0.5 pages): Table with 3-5 scenarios (Failure Mode | Detection | Degradation | Impact). 1 paragraph discussing most critical failure and mitigation.

**6. Trade-off Justifications** (0.25 pages): Summarize 2-3 major design trade-offs. Example: "Chose 640×480 over 1080p to reduce latency 150ms→50ms, accepting reduced detail for real-time performance."

### Optional Code Sketch

Not required to run, but may include ROS2 pseudocode showing subscription setup, `message_filters` synchronization, fusion logic structure (commented outline, no full implementation). This demonstrates ROS2 integration understanding from Module 2.

### Submission Guidelines

**File Naming**: `module2_capstone_[YourName].pdf` or `.md`

**Submission**: Upload to course platform per instructor.

**Academic Integrity**: Discuss scenarios and approaches with classmates, but sensor selection and architecture must be individual work. Use AI tools (Claude, ChatGPT) for brainstorming and review—encouraged. Copying designs from online sources without attribution violates academic integrity.

---

## Resources

### Module 2 Lessons

- [Lesson 1: Camera Systems](./01-camera-systems) - Camera types, resolution/frame rate trade-offs, FOV, `sensor_msgs/Image`, `CameraInfo`
- [Lesson 2: Depth Sensing](./02-depth-sensing) - LiDAR, structured light, ToF, `PointCloud2`, `LaserScan`, outdoor vs. indoor, sunlight interference
- [Lesson 3: IMU and Proprioception](./03-imu-proprioception) - Accelerometer, gyroscope, magnetometer, `sensor_msgs/Imu`, balance, drift, calibration
- [Lesson 4: Sensor Fusion](./04-sensor-fusion) - Complementary filters, Kalman filters, VIO, `robot_localization`, multi-sensor integration

### Case Studies

- **Boston Dynamics Atlas**: Multi-sensor SLAM with LiDAR + stereo + IMU for disaster response
- **Agility Robotics Digit**: VIO using stereo + IMU for GPS-free warehouse navigation
- **Tesla Optimus**: Multi-camera fusion (8+ cameras) without LiDAR, vision-only perception
- **ANYbotics ANYmal**: LiDAR + stereo + IMU for outdoor legged navigation on rough terrain

Use as conceptual references, not blueprints. Tailor designs to your scenario constraints.

### ROS2 Documentation

- **robot_localization**: [ROS2 Humble docs](http://docs.ros.org/en/humble/p/robot_localization/) - EKF for multi-sensor fusion
- **message_filters**: [ApproximateTimeSynchronizer](http://docs.ros.org/en/humble/p/message_filters/) - Timestamp matching for camera+depth+IMU
- **sensor_msgs**: [Message definitions](http://docs.ros.org/en/humble/p/sensor_msgs/) - `Image`, `PointCloud2`, `LaserScan`, `Imu`, `CameraInfo`, `NavSatFix`

### Design Tools

- **draw.io** (diagrams.net): Free web-based diagramming with ROS2 shapes
- **ROS2 rqt_graph**: Generates real-time ROS2 architecture diagrams (if ROS2 installed)
- **Lucidchart**: Web-based diagramming (free tier available)
- **Paper and pencil**: Hand-drawn diagrams fully acceptable if legible

---

## Expert Tips

### AI Colearning Prompt 1: Scenario Exploration

Before starting, ask your AI assistant:

> "I'm designing a sensor system for [chosen scenario]. Compare trade-offs between:
> 1. Stereo cameras + IMU (passive depth, lower cost)
> 2. LiDAR + monocular camera + IMU (active depth, higher cost)
>
> Consider: indoor vs. outdoor, sunlight interference, compute requirements, cost. Which would you recommend and why?"

**Learning outcome**: Explore sensor trade-offs specific to your scenario. Compare AI reasoning with Lessons 1-2 concepts to validate understanding.

### AI Colearning Prompt 2: Fusion Strategy Validation

After drafting fusion strategy, ask:

> "I'm using [fusion algorithm] to combine [sensors]. My state variables are [position, orientation, velocity, etc.]. Which sensors should contribute to which state variables, and why? Are there nonsensical sensor combinations?"

**Learning outcome**: Validate that fusion matches sensor characteristics. Cameras don't directly measure velocity (estimate via optical flow). IMUs measure acceleration/angular velocity, not position (requires integration). Understanding these prevents design mistakes.

### Expert Insight: Real-World Iteration

**Sensor Systems Are Never Perfect on First Try**

Professional teams iterate sensor configurations 5-10 times before finalizing. Boston Dynamics' Atlas went through multiple camera/LiDAR combinations over years. Tesla Optimus initially tested LiDAR before switching to camera-only for cost and form factor.

Your capstone design won't be perfect—that's expected. Demonstrate:

1. **Explicit reasoning**: Why sensor X over Y? What trade-off did you prioritize (cost vs. accuracy, latency vs. resolution)?
2. **Awareness of limitations**: What fails in your design? How do you mitigate with fusion or degradation?
3. **Realistic constraints**: Cost, compute, latency budgets force hard choices—acknowledging and working within constraints is core engineering.

Grading prioritizes justification quality over "optimal" selection. Two students can choose different sensors for the same scenario and both earn full marks if reasoning is sound and grounded in Module 2.

### Practice Exercise: ROS2 Message Synchronization

Challenge: Sketch `message_filters.ApproximateTimeSynchronizer` setup for your sensor suite.

Consider:
1. **Which sensors need synchronization?** (e.g., camera + depth for 3D object detection, camera + IMU for VIO)
2. **What slop parameter?** (Maximum time difference for "synchronized" messages). Example: Camera 15 Hz (66 ms period), IMU 100 Hz (10 ms period) → slop ~50 ms allows 1-2 camera frame drift.
3. **What if synchronization fails?** (e.g., one sensor stops publishing due to hardware failure)

Pseudocode template:
```python
from message_filters import ApproximateTimeSynchronizer, Subscriber

sync = ApproximateTimeSynchronizer(
    [camera_sub, depth_sub, imu_sub],
    queue_size=10,
    slop=0.05  # 50 ms tolerance
)
sync.registerCallback(fusion_callback)
```

**Optional**: Ask Claude to review your slop parameter choice and suggest improvements based on sensor update rates and scenario latency requirements.

---

**Good luck with your capstone design! Focus on clear justifications, realistic trade-offs, and thoughtful failure analysis. Your design documentation skills will serve you throughout your robotics career.**
