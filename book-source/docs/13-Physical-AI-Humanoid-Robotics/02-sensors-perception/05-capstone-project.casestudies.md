# Case Studies: Real-World Multi-Sensor Integration

These case studies showcase how leading humanoid robots integrate camera systems, depth sensors, IMUs, and sensor fusion to solve real-world challenges. Use these examples as design inspiration for your capstone project, not as blueprints to copy. Each robot made specific trade-offs based on their operational requirements, cost constraints, and technical priorities.

---

## 📊 Case Study 1: Boston Dynamics Atlas - Multi-Sensor SLAM for Dynamic Environments

### Robot Overview

Boston Dynamics Atlas is designed for navigation and manipulation in disaster response, construction sites, and unstructured outdoor environments. Atlas must handle rough terrain, dynamic obstacles (moving debris, humans), and perform complex whole-body manipulation tasks like lifting objects and opening doors.

### Sensor Configuration

| **Sensor Type** | **Model/Technology** | **Specifications** | **Placement** | **Update Rate** | **Estimated Cost** |
|-----------------|---------------------|-------------------|---------------|-----------------|-------------------|
| Stereo Cameras | Carnegie Robotics Multisense SL | 2MP CMOS, Bayer filter, 60° FOV | Head (pan-tilt mount) | 30 FPS | $15,000+ |
| LiDAR | Hokuyo UTM-30LX-EW (in Multisense SL) | 270° planar, 30m range, ±30mm accuracy | Head (rotating spindle at 30 RPM) | 40 Hz (1,081 points/scan) | Included in Multisense |
| RGB-D Depth Camera | Time-of-Flight (ToF) sensors | 640×480 depth resolution, 0.5-5m range | Chest/torso (fixed) | 30 Hz | $500-1,500 |
| IMU | High-precision MEMS IMU (9-axis) | Accelerometer, gyroscope, magnetometer | Torso (center of mass) | 1,000 Hz | $2,000-5,000 |
| Joint Encoders | Absolute and incremental encoders | Sub-degree accuracy, 28 DOF | All joints (28 total) | 4,000 Hz | $500-1,000 each |

**Total Sensor Suite Cost**: Approximately $25,000-35,000 (excluding joint encoders)

### Integration Approach

**Sensor Fusion Algorithm**: Atlas uses a tightly-coupled LiDAR-Inertial-Visual SLAM system that fuses:
- **LiDAR point clouds** (from rotating Hokuyo) for 3D structure mapping and obstacle detection at 5-30m range
- **Stereo camera images** for visual features, texture-based localization, and object recognition
- **IMU data** at 1 kHz for orientation tracking, motion prediction, and dynamic balance control
- **Joint encoder feedback** at 4 kHz for proprioceptive state estimation (limb positions, center-of-mass calculation)

**ROS2 Integration** (conceptual, Atlas uses proprietary middleware but principles apply):
- `/multisense/left/image_raw` (sensor_msgs/Image): Left stereo camera for visual odometry
- `/multisense/lidar_scan` (sensor_msgs/LaserScan): Planar LiDAR scan, rotated to build 3D point cloud
- `/torso/imu` (sensor_msgs/Imu): High-rate orientation, angular velocity, linear acceleration
- `/joint_states` (sensor_msgs/JointState): 28 DOF encoder positions for balance computation
- `/slam/odometry` (nav_msgs/Odometry): Fused state estimate (position, orientation, velocity) from EKF

**Key Design Decision**: The Multisense SL sensor head combines stereo cameras and a rotating LiDAR in a single unit, simplifying calibration and reducing sensor mounting complexity. The rotating LiDAR builds a 3D point cloud by sweeping a 2D scan line, achieving 360° coverage with mechanical rotation.

### Lessons Applied

**From Lesson 1 (Camera Systems)**:
- **Stereo vision** provides passive depth estimation for nearby obstacles (0.5-10m) without IR interference in outdoor sunlight
- **Active perception**: Pan-tilt head allows Atlas to "look" at objects of interest, adjusting FOV dynamically

**From Lesson 2 (Depth Sensing)**:
- **LiDAR for outdoor reliability**: Unlike RGB-D sensors, LiDAR works in direct sunlight and provides long-range (30m) obstacle detection
- **Rotating vs. solid-state**: Mechanical rotation achieves 360° coverage with a single planar LiDAR, trading mechanical complexity for cost savings vs. 3D LiDAR ($100k+)

**From Lesson 3 (IMU and Proprioception)**:
- **High-rate IMU (1 kHz)** enables dynamic balance control during parkour, backflips, and rough terrain walking
- **Joint encoders as proprioception**: 4 kHz encoder feedback provides accurate limb position for center-of-mass estimation, critical for balance

**From Lesson 4 (Sensor Fusion)**:
- **Tightly-coupled SLAM**: LiDAR, camera, and IMU data are fused in a single optimization framework, improving robustness vs. loosely-coupled systems
- **Redundancy strategy**: If camera fails (e.g., mud on lens), LiDAR + IMU provide basic navigation; if LiDAR fails, stereo vision + IMU enable short-range operation

### Design Insights

**Sensor Placement Philosophy**: Head-mounted sensors (Multisense SL) provide wide FOV and active perception via pan-tilt, but create occlusion when manipulating objects near the chest. Atlas supplements with chest-mounted ToF depth camera for "blind spot" coverage during manipulation tasks.

**Failure Handling**: Atlas prioritizes redundancy for safety-critical tasks:
- **Dual depth modalities** (stereo + LiDAR): If one fails, the other provides fallback
- **IMU drift correction**: Visual and LiDAR loop closure re-initializes IMU bias every 10-30 seconds
- **Emergency stop**: If all perception fails, IMU-only balance control attempts safe shutdown (crouch, then sit)

**Relevance to Capstone Scenarios**:
- **Scenario B (Outdoor Delivery)**: Atlas' LiDAR + stereo camera approach is ideal for outdoor navigation with long-range obstacle detection
- **Scenario D (Warehouse)**: Multi-sensor redundancy ensures high reliability for safety-critical environments

---

## 📊 Case Study 2: Agility Robotics Digit - Visual-Inertial Odometry for GPS-Free Indoor Navigation

### Robot Overview

Agility Robotics Digit is designed for warehouse logistics and last-mile delivery in indoor and semi-structured outdoor environments (sidewalks, building entrances). Digit prioritizes autonomous navigation without GPS, using Visual-Inertial Odometry (VIO) for localization. The robot must carry packages up to 16 kg while navigating through doorways, elevators, and around human workers.

### Sensor Configuration

| **Sensor Type** | **Model/Technology** | **Specifications** | **Placement** | **Update Rate** | **Estimated Cost** |
|-----------------|---------------------|-------------------|---------------|-----------------|-------------------|
| RGB-D Depth Cameras | Intel RealSense D435i (×4) | Stereo + IR depth, 1280×720 RGB, 1280×720 depth, IMU integrated | Head (forward), torso (left/right/rear) | 30 FPS (RGB), 90 FPS (depth) | $200-300 each ($800-1,200 total) |
| LiDAR | 2D planar LiDAR (model unspecified) | 270° FOV, 30m range | Chest (horizontal plane) | 10-20 Hz | $1,500-5,000 |
| IMU | Bosch BMI055 (integrated in D435i) + MEMS IMU | 6-axis (accel + gyro), ±16g, ±2000°/s | Torso (center) + cameras | 200 Hz (torso), 200 Hz (D435i) | $50-100 (integrated in D435i) |
| Absolute Encoders | Joint position sensors | Proprioceptive feedback for 20 DOF legs/arms | All leg and arm joints | 100+ Hz | $300-500 each |

**Total Sensor Suite Cost**: Approximately $5,000-10,000 (excluding encoders)

### Integration Approach

**Sensor Fusion Algorithm**: Digit uses **Visual-Inertial Odometry (VIO)** combining:
- **Four Intel RealSense D435i cameras** provide 360° RGB-D coverage and integrated IMU data
- **Stereo visual features** from multiple cameras enable robust visual odometry with redundancy (if one camera fails, others compensate)
- **IMU data** fuses with visual odometry to handle rapid motion, camera occlusion, and reduce drift during dynamic walking
- **LiDAR** supplements for obstacle detection and loop closure detection in large warehouses

**ROS2 Integration** (Digit uses proprietary software, but ROS2 equivalent would be):
- `/camera/front/color/image_raw`, `/camera/left/color/image_raw`, etc. (sensor_msgs/Image): 4 cameras for multi-view VIO
- `/camera/front/depth/image_rect_raw` (sensor_msgs/Image): Depth images for obstacle avoidance
- `/camera/front/imu` (sensor_msgs/Imu): Integrated IMU from D435i cameras
- `/torso/imu` (sensor_msgs/Imu): High-precision torso IMU for balance control
- `/scan` (sensor_msgs/LaserScan): 2D LiDAR for obstacle detection
- `/odometry/vio` (nav_msgs/Odometry): Fused VIO output combining cameras + IMU

**Key Design Decision**: Digit uses **four RealSense D435i cameras** instead of a single high-end camera or 3D LiDAR. This provides:
1. **Redundancy**: If one camera's view is blocked (e.g., carrying a large package), others maintain localization
2. **360° coverage**: Front, left, right, rear cameras eliminate blind spots
3. **Cost efficiency**: Four $250 cameras ($1,000 total) vs. one 3D LiDAR ($8,000+)

### Lessons Applied

**From Lesson 1 (Camera Systems)**:
- **Multi-camera visual odometry**: Tracks features across overlapping camera views, increasing robustness vs. single stereo pair
- **RGB-D integration**: D435i combines RGB camera with active IR stereo depth, enabling operation in low-light warehouses

**From Lesson 2 (Depth Sensing)**:
- **Active IR stereo**: D435i projects IR pattern for texture-less surfaces (white walls, uniform floors), overcoming passive stereo limitations
- **Depth + LiDAR complementarity**: RGB-D provides dense depth (0.3-3m) for close-range manipulation; LiDAR provides sparse long-range (5-30m) for navigation

**From Lesson 3 (IMU and Proprioception)**:
- **VIO requires tight camera-IMU synchronization**: D435i integrates IMU on camera PCB, ensuring hardware time-synchronization (&lt;1 ms jitter)
- **Balance control vs. navigation IMU**: Torso IMU (200 Hz) for dynamic balance; camera IMUs (200 Hz) for VIO

**From Lesson 4 (Sensor Fusion)**:
- **VIO (Visual-Inertial Odometry)**: Fuses visual features with IMU using Extended Kalman Filter (EKF) or graph optimization (e.g., VINS-Mono algorithm)
- **Loop closure**: When Digit revisits a location, visual recognition triggers re-localization, correcting accumulated drift

### Design Insights

**GPS-Free Localization Philosophy**: Digit operates in environments where GPS is unavailable (indoor warehouses, building interiors). VIO provides drift-bounded localization:
- **Short-term (&lt;1 min)**: IMU-predicted motion between camera frames (&lt;0.1% drift)
- **Medium-term (1-10 min)**: Visual odometry bounds IMU drift to &lt;1% of distance traveled
- **Long-term (10+ min)**: Loop closure re-initializes position when revisiting known areas

**Failure Handling**:
- **Single camera failure**: VIO continues with remaining 3 cameras at reduced accuracy (±5 cm → ±10 cm localization error)
- **IMU drift**: If visual features are lost (e.g., uniform white hallway), IMU dead-reckoning for &lt;5 seconds until features reappear
- **Complete vision failure**: Fall back to joint encoders + IMU for basic balance, stop autonomous navigation, request manual intervention

**Sensor Placement Strategy**: RealSense cameras placed at torso height (not head) to minimize occlusion during package carrying. Forward and side cameras provide frontal FOV overlap for robust stereo matching.

**Relevance to Capstone Scenarios**:
- **Scenario A (Home Assistant)**: Digit's VIO approach ideal for GPS-free indoor navigation with cost-effective RGB-D cameras
- **Scenario C (Human Interaction)**: Multi-camera setup provides 360° awareness for safe human proximity
- **Scenario D (Warehouse)**: Proven VIO in real warehouse deployments (Amazon, GXO Logistics)

---

## 📊 Case Study 3: Tesla Optimus - Multi-Camera Fusion for Cost-Optimized Manipulation

### Robot Overview

Tesla Optimus (also known as Tesla Bot) is designed for general-purpose humanoid tasks in factories, homes, and service environments. Optimus prioritizes cost reduction by leveraging Tesla's automotive Full Self-Driving (FSD) computer and camera-only perception (no LiDAR). The robot must perform manipulation tasks (assembly, picking, placing) and navigate indoor/outdoor environments.

### Sensor Configuration

| **Sensor Type** | **Model/Technology** | **Specifications** | **Placement** | **Update Rate** | **Estimated Cost** |
|-----------------|---------------------|-------------------|---------------|-----------------|-------------------|
| Head Cameras | 8× automotive-grade cameras | 1280×960 resolution, wide/standard/narrow FOV mix | Head (3× forward, 2× side, 1× rear, 2× downward) | 30-60 FPS | $50-200 each ($400-1,600 total) |
| Wrist Cameras | 2× high-resolution cameras | 1920×1080 or higher for manipulation | Wrists (gripper-mounted) | 30 FPS | $100-300 each ($200-600 total) |
| IMU | 6-axis MEMS IMU | ±8g accel, ±1000°/s gyro | Torso (center of mass) | 100-200 Hz | $20-50 |
| Joint Force/Torque Sensors | Actuator-integrated sensors | Measure joint torque for compliant control | All 28+ joints | 200+ Hz | Integrated in actuators |
| Foot Force Sensors | Pressure sensors or load cells | Ground contact force for gait stability | Both feet | 200+ Hz | $50-200 each |

**Total Sensor Suite Cost**: Approximately $1,000-3,000 (excluding actuator-integrated sensors)

**Notable Omission**: **No LiDAR or dedicated depth sensors**. Depth estimated via monocular depth prediction neural networks.

### Integration Approach

**Sensor Fusion Algorithm**: Optimus uses **multi-camera neural network fusion** running on Tesla FSD computer:
- **Eight automotive cameras** provide overlapping 360° coverage for navigation and obstacle avoidance
- **Monocular depth estimation**: Neural networks trained on millions of driving scenarios predict depth from single camera images
- **Multi-camera occupancy grid**: Fuses 8 camera views into 3D occupancy map (similar to Tesla's BEV - Bird's Eye View - representation for cars)
- **Wrist cameras** for manipulation: Visual servoing during grasping, object pose estimation
- **IMU + vision fusion**: IMU provides motion prediction between camera frames; neural network compensates for camera motion

**ROS2 Integration** (conceptual for educational purposes):
- `/camera/head_forward_wide/image_raw`, `/camera/head_forward_narrow/image_raw`, etc. (sensor_msgs/Image): 8 head cameras
- `/camera/wrist_left/image_raw`, `/camera/wrist_right/image_raw` (sensor_msgs/Image): Gripper-mounted cameras
- `/imu/data` (sensor_msgs/Imu): Torso IMU for balance and motion prediction
- `/joint_states` (sensor_msgs/JointState): Actuator positions and torques
- `/depth/prediction` (sensor_msgs/Image): Neural network monocular depth prediction (not sensor_msgs/PointCloud2, as depth is estimated, not measured)
- `/perception/occupancy_grid` (nav_msgs/OccupancyGrid): Fused 3D obstacle map from multi-camera neural network

**Key Design Decision**: **Vision-only perception** (no LiDAR) reduces sensor cost from $8,000-20,000 (3D LiDAR) to &lt;$2,000 (cameras only). Trade-off: depth estimation is less accurate (±10-20 cm at 5m vs. ±2 cm for LiDAR) but sufficient for navigation and manipulation tasks in structured environments.

### Lessons Applied

**From Lesson 1 (Camera Systems)**:
- **Multi-camera stereo**: Overlapping FOV cameras enable stereo depth triangulation (e.g., forward-wide + forward-narrow cameras)
- **Wrist-mounted cameras**: Eye-in-hand configuration provides unoccluded view of grasped objects until contact

**From Lesson 2 (Depth Sensing)**:
- **Monocular depth neural networks**: Trained on massive datasets (Tesla FSD data), predict depth without structured light or ToF hardware
- **Trade-off acceptance**: Lower depth accuracy acceptable for navigation (collision avoidance requires ~10 cm precision, not &lt;1 cm)

**From Lesson 3 (IMU and Proprioception)**:
- **Force/torque sensing**: Joint actuators measure applied forces, enabling compliant manipulation (e.g., gentle handoff, adaptive grip)
- **Foot force sensors**: Ground reaction forces for zero-moment point (ZMP) balance control

**From Lesson 4 (Sensor Fusion)**:
- **Neural network as fusion algorithm**: Instead of traditional EKF/UKF, Tesla uses transformers to fuse 8 camera views + IMU into unified 3D occupancy representation
- **Temporal fusion**: Neural network processes camera sequences (not single frames), implicitly learning visual odometry

### Design Insights

**Cost Reduction Philosophy**: Optimus targets consumer/factory price points ($20,000-30,000 estimated), requiring aggressive sensor cost reduction:
- **No LiDAR**: Saves $8,000-20,000 per robot
- **Automotive-grade cameras**: Leverage Tesla's supply chain ($50-200 per camera vs. $500-2,000 for robotics-grade cameras)
- **FSD computer reuse**: Amortizes R&D costs across automotive and robotics platforms

**Wrist Camera Benefits**:
- **Unoccluded view**: Unlike head cameras, wrist cameras maintain view of object until gripper closes
- **Close-range precision**: 1080p cameras at 0.2-0.5m provide sub-millimeter pixel resolution for grasp refinement
- **Lighting control**: Wrist-mounted LED ring illuminates workspace, reducing sensitivity to ambient lighting

**Failure Handling**:
- **Camera redundancy**: 8 head cameras mean 1-2 camera failures still allow degraded navigation (7 cameras → reduced FOV coverage)
- **Monocular depth uncertainty**: Neural network outputs confidence scores; low-confidence depth estimates trigger cautious behavior (slow down, use other cameras)
- **IMU drift**: Visual odometry from multiple cameras re-initializes IMU bias every 1-2 seconds

**Computational Strategy**: FSD computer (custom ASIC) runs neural networks at 144 TOPS (tera operations per second), enabling real-time processing of 8 camera streams + depth prediction + object detection. Lower-cost robots cannot afford this compute power, making LiDAR + simpler algorithms more practical.

**Relevance to Capstone Scenarios**:
- **Scenario A (Home Assistant)**: Vision-only approach viable for indoor tasks if compute budget allows neural network depth estimation
- **Scenario C (Human Interaction)**: Wrist cameras essential for safe object handoff and manipulation
- **Cost-constrained designs**: Demonstrates trade-off between sensor hardware (LiDAR) vs. computation (neural networks for depth estimation)

---

## 📊 Case Study 4: Comparative Analysis - Sensor Suite Trade-offs

This table summarizes key design decisions across the three case studies, highlighting how different operational requirements drive sensor selection.

| **Criterion** | **Atlas (Boston Dynamics)** | **Digit (Agility Robotics)** | **Optimus (Tesla)** |
|---------------|----------------------------|----------------------------|---------------------|
| **Primary Environment** | Outdoor/disaster (unstructured) | Indoor warehouse (semi-structured) | Indoor factory/home (structured) |
| **Depth Technology** | LiDAR (rotating 2D) + Stereo | RGB-D (active IR stereo) + LiDAR | Monocular depth neural networks |
| **Depth Sensor Cost** | $15,000+ (Multisense SL) | $1,000-1,500 (4× RealSense D435i + LiDAR) | $0 (cameras only, depth estimated) |
| **Camera Count** | 2 (stereo pair) | 8 (4× RGB-D cameras = 8 image streams) | 10 (8 head + 2 wrist) |
| **IMU Update Rate** | 1,000 Hz (high-precision) | 200 Hz (torso + integrated) | 100-200 Hz (standard MEMS) |
| **Fusion Algorithm** | Tightly-coupled LiDAR-Visual-Inertial SLAM | VIO (Visual-Inertial Odometry) with loop closure | Multi-camera neural network fusion (transformer-based) |
| **Localization Drift** | &lt;0.1% (LiDAR + loop closure) | &lt;1% (VIO with loop closure) | &lt;2% (monocular depth less accurate) |
| **Compute Platform** | Custom high-performance CPU/GPU | Dual Intel i7 CPUs + optional Jetson | Tesla FSD computer (144 TOPS custom ASIC) |
| **Estimated Sensor Cost** | $25,000-35,000 | $5,000-10,000 | $1,000-3,000 |
| **Cost Optimization** | Performance-first (disaster response requires reliability) | Balanced (warehouse profitability requires cost control) | Cost-first (consumer/factory scale requires low price) |
| **Failure Redundancy** | Dual depth modalities (stereo + LiDAR) | Multi-camera redundancy (4 cameras, lose 1-2 OK) | 8 head cameras, partial failure tolerated |
| **Outdoor Sunlight** | LiDAR handles sunlight well | RGB-D IR pattern may wash out, relies on LiDAR fallback | Monocular depth neural networks handle sunlight (trained on outdoor driving) |
| **Applicable Capstone Scenarios** | Scenario B (Outdoor Delivery), Scenario D (Warehouse) | Scenario A (Home Assistant), Scenario D (Warehouse) | Scenario A (Home Assistant), Scenario C (Human Interaction) |

### Key Takeaway for Capstone Design

**There is no "best" sensor suite—only appropriate trade-offs for specific scenarios**:

1. **If outdoor operation is required** (Scenario B): LiDAR is nearly mandatory due to sunlight interference with IR-based RGB-D sensors. Atlas demonstrates this principle.

2. **If indoor GPS-free navigation is required** (Scenarios A, C, D): VIO (camera + IMU fusion) provides drift-bounded localization. Digit proves this works at commercial scale.

3. **If cost is paramount and compute is available** (Scenario A with consumer budget): Monocular depth estimation can replace hardware depth sensors, as Optimus demonstrates. However, this requires significant neural network training and inference compute.

4. **If manipulation precision is critical** (Scenarios A, C): Wrist-mounted cameras (Optimus approach) provide unoccluded view of objects during grasping. Head cameras alone create blind spots when arms reach forward.

5. **If safety is critical** (Scenario C: human interaction): Redundant depth sensing (e.g., Atlas' stereo + LiDAR) ensures zero collision risk even if one sensor fails.

When designing your capstone project, **justify sensor choices with explicit trade-offs** (cost vs. accuracy, indoor vs. outdoor, compute vs. hardware). Reference these case studies to demonstrate awareness of real-world design decisions.

---

## Summary: Applying Case Studies to Your Capstone

Use these case studies as **conceptual references**, not blueprints:

1. **Atlas shows**: When reliability and outdoor operation are paramount, invest in LiDAR and redundant sensors (Scenarios B, D)
2. **Digit shows**: VIO with multiple RGB-D cameras is viable for indoor logistics at moderate cost (Scenarios A, D)
3. **Optimus shows**: Vision-only approaches can work if neural network compute is available and accuracy requirements are relaxed (Scenarios A, C)

**Design Process**:
1. Identify your scenario constraints (indoor vs. outdoor, cost budget, precision requirements)
2. Map constraints to sensor technologies from Lessons 1-4
3. Reference case studies for real-world validation (e.g., "Digit uses VIO for warehouse navigation, so VIO is appropriate for Scenario D")
4. Justify trade-offs explicitly (e.g., "Chose stereo cameras over RGB-D because Scenario B requires outdoor operation where sunlight interferes with IR depth sensors, as seen in Atlas' LiDAR-first design")

**What Makes a Strong Capstone Design**:
- Sensor choices **reference specific limitations** from Lessons 1-4 (e.g., "RGB-D fails on glass surfaces, Lesson 2 Section 3.3")
- Fusion strategy **matches sensor characteristics** (e.g., "VIO appropriate for camera + IMU, not LiDAR + magnetometer")
- Failure modes **demonstrate awareness** of real-world issues (e.g., "Atlas handles camera mud occlusion with LiDAR fallback")
- ROS2 architecture is **realistic and implementable** (message types, topic names, QoS policies correct)

These case studies provide the **evidence base** for your design justifications. When explaining sensor selection, cite these robots as proof that your approach works in practice.
