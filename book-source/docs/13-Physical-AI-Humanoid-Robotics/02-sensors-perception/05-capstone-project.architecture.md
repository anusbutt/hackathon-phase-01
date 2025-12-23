# Multi-Sensor Fusion Capstone: Architecture Documentation

## System Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    MULTI-SENSOR CAPSTONE ARCHITECTURE                    │
└─────────────────────────────────────────────────────────────────────────┘

                         SENSOR HARDWARE LAYER
    ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
    │  Camera Driver  │  │ Depth Sensor    │  │   IMU Driver    │
    │  (USB/CSI)      │  │ (RealSense/     │  │  (BNO055/MPU)   │
    │                 │  │  Kinect/LiDAR)  │  │                 │
    └────────┬────────┘  └────────┬────────┘  └────────┬────────┘
             │                    │                    │
             │ /camera/image_raw  │ /camera/depth/     │ /imu/data
             │ (sensor_msgs/      │  points            │ (sensor_msgs/
             │  Image)            │ (sensor_msgs/      │  Imu)
             │                    │  PointCloud2)      │
             ▼                    ▼                    ▼
    ┌────────────────────────────────────────────────────────────┐
    │         MESSAGE_FILTERS: TIME SYNCHRONIZATION              │
    │  ApproximateTimeSynchronizer(queue=10, slop=0.1s)          │
    └────────────────────────┬───────────────────────────────────┘
                             │
                             │ Synchronized Callback
                             │
                             ▼
    ┌────────────────────────────────────────────────────────────┐
    │        MULTI-SENSOR CAPSTONE NODE (Fusion Core)            │
    │                                                             │
    │  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐  │
    │  │ Camera        │  │ Depth         │  │ IMU           │  │
    │  │ Processor     │  │ Processor     │  │ Processor     │  │
    │  │ (Lesson 1)    │  │ (Lesson 2)    │  │ (Lesson 3)    │  │
    │  └───────┬───────┘  └───────┬───────┘  └───────┬───────┘  │
    │          │                  │                  │           │
    │          └──────────┬───────┴──────────────────┘           │
    │                     │                                      │
    │           ┌─────────▼──────────┐                           │
    │           │  Fusion Algorithm  │                           │
    │           │    (Lesson 4)      │                           │
    │           │ • Complementary    │                           │
    │           │ • Kalman Filter    │                           │
    │           │ • Weighted Average │                           │
    │           └─────────┬──────────┘                           │
    └─────────────────────┼──────────────────────────────────────┘
                          │
                          │
            ┌─────────────┴──────────────┐
            │                            │
            ▼                            ▼
    /fused_pose                   /fused_odom
    (geometry_msgs/               (nav_msgs/
     PoseStamped)                  Odometry)
            │                            │
            └─────────────┬──────────────┘
                          │
                          ▼
    ┌────────────────────────────────────────────────────────────┐
    │              DOWNSTREAM APPLICATIONS                        │
    │  • Robot Control (balance, navigation)                     │
    │  • Safety Monitoring (fall detection, collision avoidance) │
    │  • Visualization (RViz)                                    │
    │  • Data Logging & Analysis                                 │
    └────────────────────────────────────────────────────────────┘
```

---

## Data Flow Pipeline

### 1. Sensor Acquisition (Parallel Streams)

| Sensor | Message Type | Frequency | QoS Profile | Purpose |
|--------|--------------|-----------|-------------|---------|
| Camera | sensor_msgs/Image | 30 Hz | BEST_EFFORT | Visual features, object detection |
| Depth | sensor_msgs/PointCloud2 | 30 Hz | BEST_EFFORT | Spatial awareness, obstacle distance |
| IMU | sensor_msgs/Imu | 100 Hz | RELIABLE | Orientation, acceleration, angular velocity |

### 2. Time Synchronization

```
message_filters.ApproximateTimeSynchronizer
├── Queue Size: 10 messages per sensor
├── Slop Tolerance: 0.1 seconds (configurable)
└── Callback: Triggered when all 3 sensors aligned within tolerance
```

**Key Decision**: ApproximateTimeSynchronizer chosen over ExactTimeSynchronizer
- **Rationale**: Real sensors have clock drift and variable latency
- **Tradeoff**: Small temporal misalignment (≤100ms) vs missing data
- **Alternative**: ExactTimeSynchronizer for simulation with perfect clocks

### 3. Sensor Processing (Modular Design)

Each sensor has dedicated processing method:

```python
_process_camera()  → visual_features (dict)
_process_depth()   → spatial_info (dict)
_process_imu()     → motion_state (dict)
```

**Extension Pattern**: Students implement scenario-specific logic in these methods
- Camera: Object detection, visual SLAM features
- Depth: Obstacle detection, ground segmentation
- IMU: Balance monitoring, motion classification

### 4. Fusion Algorithm (Customizable Core)

```python
_fuse_sensors(visual, spatial, motion) → fused_state (dict)
```

**Fusion Strategies** (students choose based on scenario):

| Strategy | Use Case | Complexity | Accuracy |
|----------|----------|------------|----------|
| Weighted Average | Simple fusion, confidence-based | Low | Moderate |
| Complementary Filter | Orientation fusion (IMU + visual) | Medium | Good |
| Extended Kalman Filter | Full state estimation (pose + velocity) | High | Excellent |
| Visual-Inertial Odometry | Navigation, localization | Very High | Excellent |

### 5. Output Publication

- **PoseStamped**: Position + orientation (for control)
- **Odometry**: Full state (pose + velocity, optional)

---

## ROS2 Concepts Demonstrated

### Core ROS2 Features

1. **Node Architecture**: Object-oriented node design with `rclpy.node.Node`
2. **Publishers & Subscribers**: Multi-topic communication
3. **Message Filters**: Time synchronization for multi-sensor fusion
4. **QoS Profiles**: Reliability vs performance tradeoffs
5. **Parameters**: Runtime configurability without code changes
6. **Launch System**: Orchestrating multiple nodes with Python launch files
7. **YAML Configuration**: Centralized parameter management

### Advanced Patterns

1. **Separation of Concerns**:
   - Sensor drivers (hardware interface)
   - Processing logic (computation)
   - Fusion algorithm (integration)
   - Output publishing (communication)

2. **Modularity**:
   - Each sensor processor is independent
   - Fusion algorithm is swappable
   - Configuration externalized to YAML

3. **Extensibility**:
   - Clear TODO markers for student customization
   - Dictionary-based interfaces (easy to add fields)
   - Scenario-specific parameters in config file

---

## Connections to Course Lessons

### Lesson 1: Camera Systems
- **Template Integration**: `_process_camera()` method
- **Concepts Applied**:
  - Image message handling (sensor_msgs/Image)
  - Visual feature extraction placeholders
  - Object detection integration points
- **Student Extensions**:
  - YOLO object detection
  - Visual SLAM features (ORB, SIFT)
  - Lane detection for navigation
  - Gesture recognition

### Lesson 2: Depth Sensing
- **Template Integration**: `_process_depth()` method
- **Concepts Applied**:
  - PointCloud2 message processing
  - Obstacle distance computation
  - Spatial awareness for navigation
- **Student Extensions**:
  - Ground plane segmentation
  - 3D object localization
  - Occupancy grid generation
  - Collision avoidance logic

### Lesson 3: IMU & Proprioception
- **Template Integration**: `_process_imu()` method
- **Concepts Applied**:
  - Quaternion orientation handling
  - Angular velocity and linear acceleration
  - Motion state estimation
- **Student Extensions**:
  - Balance monitoring (humanoid robots)
  - Fall detection algorithms
  - Motion classification (walk/stand/run)
  - Tilt angle computation for stability

### Lesson 4: Sensor Fusion
- **Template Integration**: `_fuse_sensors()` method
- **Concepts Applied**:
  - Multi-modal data integration
  - Confidence-weighted fusion
  - Synchronized processing pipeline
- **Student Extensions**:
  - Complementary filter (IMU + visual orientation)
  - Extended Kalman Filter (state estimation)
  - Visual-Inertial Odometry (VIO)
  - Bayesian fusion with uncertainty

---

## Student Adaptation Guide

### Scenario 1: Humanoid Balance Control

**Modifications**:
1. **IMU Processing**: Implement tilt angle computation from quaternion
2. **Depth Processing**: Detect ground plane for stance stability
3. **Fusion**: Complementary filter combining IMU orientation with visual correction
4. **Output**: Publish joint control commands to maintain balance

**Config Changes**:
```yaml
balance_control:
  max_tilt_angle: 15.0
  imu_orientation_weight: 0.7
  enable_fall_detection: true
```

### Scenario 2: Obstacle Avoidance Navigation

**Modifications**:
1. **Camera Processing**: Detect obstacles using object detection (YOLO)
2. **Depth Processing**: Compute minimum obstacle distance
3. **Fusion**: Choose nearest obstacle from visual + depth sources
4. **Output**: Publish velocity commands (Twist) for safe navigation

**Config Changes**:
```yaml
obstacle_avoidance:
  safe_distance: 0.5
  fusion_strategy: "depth_priority"
```

### Scenario 3: Visual-Inertial Odometry (VIO)

**Modifications**:
1. **Camera Processing**: Extract visual features (ORB) and track between frames
2. **Depth Processing**: Provide scale correction for monocular visual odometry
3. **IMU Processing**: Predict pose changes using angular velocity
4. **Fusion**: EKF combining visual motion estimates with IMU predictions

**Config Changes**:
```yaml
visual_inertial_odometry:
  max_features: 200
  enable_depth_scale: true
  measurement_noise_visual: 0.1
```

---

## Key Architectural Decisions

### Decision 1: Message Filters for Synchronization
- **Alternatives Considered**: Manual timestamp matching, callback-based sync
- **Rationale**: message_filters provides tested, reliable time alignment
- **Tradeoff**: Adds latency (queue buffering) but guarantees alignment

### Decision 2: Dictionary-Based Interfaces
- **Alternatives Considered**: Typed dataclasses, ROS messages for internal data
- **Rationale**: Flexibility for students to add scenario-specific fields
- **Tradeoff**: Less type safety, but easier to extend

### Decision 3: Separate Processing Methods
- **Alternatives Considered**: Single unified callback with inline processing
- **Rationale**: Modularity allows students to focus on one sensor at a time
- **Tradeoff**: More function calls, but clearer code organization

### Decision 4: Configuration via YAML
- **Alternatives Considered**: Hard-coded constants, command-line arguments
- **Rationale**: Industry best practice, supports multiple scenarios without recompilation
- **Tradeoff**: Additional file to manage, but promotes good practices

---

## Testing & Validation Checklist

### Integration Tests
- [ ] Camera topic receives messages at expected rate
- [ ] Depth topic receives messages at expected rate
- [ ] IMU topic receives messages at expected rate
- [ ] Synchronized callback triggers only when all 3 sensors available
- [ ] Fusion output published within 100ms of sensor data arrival

### Functional Tests
- [ ] Camera processing extracts expected features
- [ ] Depth processing computes obstacle distances
- [ ] IMU processing converts quaternions correctly
- [ ] Fusion algorithm produces reasonable outputs
- [ ] Output messages have correct timestamp and frame_id

### Performance Tests
- [ ] Fusion node maintains target rate (30 Hz)
- [ ] Memory usage stable over 1-hour run
- [ ] No message queue overflows or dropped data
- [ ] CPU usage within acceptable limits (&lt;50% single core)

### Educational Tests
- [ ] Students can modify camera processing without breaking fusion
- [ ] Students can swap fusion algorithms via config changes
- [ ] Code comments clearly explain ROS2 concepts
- [ ] Launch file successfully starts all nodes on first try

---

## Future Extensions

1. **Advanced Synchronization**: Adaptive slop based on sensor latency
2. **Multi-Rate Fusion**: Handle sensors with different frequencies
3. **Fault Tolerance**: Graceful degradation if one sensor fails
4. **Performance Profiling**: Built-in timing diagnostics
5. **Replay Capability**: Record/playback sensor data for debugging
6. **Web Dashboard**: Real-time visualization of fusion metrics

---

## Resources for Students

### ROS2 Documentation
- [message_filters Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html)
- [Launch Files Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [QoS Profiles](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)

### Sensor Fusion Algorithms
- Complementary Filter: [Blog Post](https://www.pieter-jan.com/node/11)
- Extended Kalman Filter: [ROS2 robot_localization](https://github.com/cra-ros-pkg/robot_localization)
- Visual-Inertial Odometry: [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)

### Example Capstone Projects
1. Two-wheeled balancing robot (Segway-style)
2. Autonomous wheelchair navigation
3. Drone visual-inertial navigation
4. Humanoid walking stabilization
5. Mobile manipulator obstacle avoidance
