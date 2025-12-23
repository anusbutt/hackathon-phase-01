---
id: urdf-humanoid-basics
title: "URDF for Humanoid Robots"
sidebar_position: 4
description: "Learn how robots are described structurally using URDF (Unified Robot Description Format) to define humanoid robot bodies, joints, and links for simulation, visualization, and motion planning."
time_estimate: "60 minutes"
difficulty_level: "beginner"
prerequisites: ["01-ros2-fundamentals", "02-nodes-topics-services", "03-python-rclpy-bridge", "Basic understanding of XML syntax"]
related_lessons: ["01-ros2-fundamentals", "02-nodes-topics-services", "03-python-rclpy-bridge"]
assessment_method: "quiz"
skills:
  - name: "URDF Syntax Understanding"
    proficiency_level: "beginner"
    category: "robot-modeling"
    bloom_level: "understand"
    digcomp_area: "technical-concepts"
    measurable_at_this_level: "read a simple URDF file and identify links, joints, parent-child relationships, and joint types"
  - name: "Robot Modeling Concepts"
    proficiency_level: "beginner"
    category: "system-design"
    bloom_level: "understand"
    digcomp_area: "technical-concepts"
    measurable_at_this_level: "explain what URDF represents conceptually and why it's essential for simulation and motion planning"
learning_objectives:
  - objective: "Understand what URDF is and how it describes robot structure"
    proficiency_level: "beginner"
    bloom_level: "understand"
    assessment_method: "quiz question 12"
  - objective: "Identify links (body parts) and joints (connections) in URDF code"
    proficiency_level: "beginner"
    bloom_level: "apply"
    assessment_method: "quiz question 13 (code reading)"
  - objective: "Explain the kinematic tree concept and parent-child relationships in robot models"
    proficiency_level: "beginner"
    bloom_level: "understand"
    assessment_method: "practice exercise + capstone project"
  - objective: "Recognize how URDF enables simulation, visualization, and motion planning"
    proficiency_level: "beginner"
    bloom_level: "understand"
    assessment_method: "quiz + capstone project"
cognitive_load:
  new_concepts: 4
  assessment: "moderate - builds on ROS2 fundamentals and introduces robot modeling, requires understanding XML structure"
differentiation:
  extension_for_advanced: "Explore URDF visual and collision meshes using STL/DAE files. Investigate Xacro (XML macros) for parameterized URDF generation. Research SDF (Simulation Description Format) for advanced Gazebo simulations."
  remedial_for_struggling: "Review basic XML syntax (tags, attributes, nesting) and tree data structures (parent-child relationships) before continuing."
tags: ["ros2", "urdf", "robot-modeling", "simulation", "gazebo", "rviz", "kinematics", "xml"]
generated_by: "manual"
created: "2025-12-06"
last_modified: "2025-12-06"
ros2_version: "humble"
---

# URDF for Humanoid Robots

Understanding how robots are described as structured data—learning the URDF format that transforms mechanical designs into digital models for simulation, visualization, and intelligent control.

## What Is URDF?

**URDF** (Unified Robot Description Format) is an XML-based format for describing the physical structure of a robot. Think of URDF as the blueprint that tells ROS2 (and simulation tools like Gazebo) exactly what your robot looks like: which body parts exist, how they connect, what they can do, and where they are in space.

Unlike code that describes behavior (like the Python rclpy nodes you wrote in Lesson 3), URDF describes **structure**. It's a declarative format: you specify "the robot has a torso link connected to an upper arm link via a shoulder joint that rotates" without writing any procedural logic. Simulation engines, motion planners, and visualization tools read this URDF file and automatically understand your robot's capabilities and constraints.

At its core, URDF defines two fundamental concepts: **links** and **joints**. Links represent rigid body parts (like a forearm, thigh, or sensor housing). Joints represent connections between links that define how they move relative to each other (like a hinge for an elbow or a ball joint for a shoulder). By combining links and joints, URDF builds a **kinematic tree**—a hierarchical structure that mirrors how body parts connect in real robots.

**URDF is XML**, which means it uses tagged elements like `<link>`, `<joint>`, and `<visual>` with nested attributes. If you've worked with HTML, XML will feel familiar—it's a structured way to represent hierarchical data. For roboticists, this XML structure makes URDF human-readable (you can open it in a text editor and understand the robot's structure) while also being machine-parsable (tools can automatically load and interpret it).

What makes URDF particularly powerful is its **ecosystem integration**. When you create a URDF file for your humanoid robot, it automatically works with RViz (ROS2's 3D visualizer), Gazebo (physics simulator), MoveIt (motion planning library), and countless other ROS2 tools. You define your robot once in URDF, and the entire ROS2 ecosystem can work with it—no custom integration code needed.

URDF has become the standard robot description format in the ROS community. When you read research papers about humanoid robots, browse open-source robot repositories, or integrate third-party libraries, you'll encounter URDF files. Learning to read and understand URDF means you can quickly grasp any robot's structure, whether it's a simple arm or a complex humanoid.

## Why URDF Matters for Physical AI

Building a humanoid robot isn't just about writing control software—you need to model the robot's physical structure so that planning algorithms understand what movements are possible, simulators can test behaviors safely, and visualization tools can display what the robot is doing in real-time.

**Simulation Before Reality**: Humanoid robots are expensive and fragile. Before you command a real robot to attempt a walking gait or reaching motion, you test in simulation. Gazebo (the standard ROS2 physics simulator) reads your URDF file and creates a digital twin of your robot with accurate physics—gravity, inertia, collisions, friction. You can experiment with control algorithms, test edge cases, and iterate rapidly without risk of hardware damage. Without URDF, you'd need to manually code the simulator's understanding of your robot's structure—weeks of work that URDF reduces to a single file.

**Motion Planning Depends on Structure**: When you want your humanoid to reach for an object, motion planning algorithms (like MoveIt) must know which joints can move, their ranges of motion, and how moving one joint affects the positions of connected links. URDF provides this kinematic information precisely. The planner reads your URDF, builds an internal model of the robot's degrees of freedom, and computes collision-free trajectories. This is why professional robot manipulation systems universally use URDF—it's the data format that motion planning libraries expect.

**Visualization for Debugging**: When developing robot behaviors, you need to see what your robot is doing. RViz (ROS Visualization) reads URDF files and renders 3D models of your robot in real-time, updating joint positions as your robot moves. This visualization is invaluable for debugging—if your walking controller has a bug that causes the robot to cross its legs, you'll see it immediately in RViz before deploying to hardware. URDF's support for visual meshes (3D models like STL files) means your visualization can look exactly like the real robot.

**Standardization Enables Collaboration**: Because URDF is the de facto standard, you can leverage pre-built robot models from the community. Need to test your AI algorithms on a humanoid? Download a URDF model of a PR2, Atlas, or Nao robot and start immediately. Collaborating with a team? Share your URDF file, and everyone's simulators and visualizers work identically. This standardization dramatically accelerates development—you're not reinventing infrastructure; you're building on decades of community effort.

**Hardware Abstraction**: URDF creates a layer between your high-level planning code and the specific hardware. Whether your humanoid uses servo motors, hydraulic actuators, or direct-drive joints, the URDF representation is the same: joints with position limits and kinematic relationships. This abstraction means you can develop and test motion planning algorithms in simulation, then deploy to different robot hardware with minimal code changes.

## Key Principles

### Links: Robot Body Parts

In URDF, a **link** represents a single rigid body part of your robot. For a humanoid, links include the torso, upper arm, forearm, thigh, shin, foot, head, etc. Each link can have visual properties (how it looks), collision properties (its shape for collision detection), and inertial properties (mass, center of mass, inertia for physics simulation).

At minimum, a link needs a name:

```xml
<link name="torso">
  <!-- Visual, collision, and inertial properties go here -->
</link>
```

For visualization and simulation, you typically add visual geometry (simplified shapes like boxes, cylinders, or 3D mesh files):

```xml
<link name="upper_arm">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </visual>
</link>
```

This defines an upper arm as a cylinder with 5cm radius and 30cm length, positioned so its base is at the origin. The `<origin>` tag uses **xyz** (position in meters) and **rpy** (roll-pitch-yaw orientation in radians)—standard URDF units.

### Joints: Connections Between Links

A **joint** connects two links and defines how they move relative to each other. Every joint has a **parent link** (the stationary reference) and a **child link** (the moving part). Joints specify the type of motion allowed—revolute (hinge rotation), prismatic (sliding), continuous (unlimited rotation), fixed (no motion), etc.

```xml
<joint name="shoulder" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0 0.2 0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
</joint>
```

This shoulder joint:
- Connects the torso (parent) to the upper arm (child)
- Is a **revolute** joint (hinge that rotates)
- Is positioned 20cm to the side and 40cm up from the torso's origin
- Rotates around the Y-axis (`<axis xyz="0 1 0"/>`)
- Has limits: -90° to +90° rotation (in radians), max torque 100Nm, max velocity 1.0 rad/s

### Kinematic Trees: Parent-Child Hierarchies

URDF robots form a **tree structure** where one link is the root (typically `base_link` or `torso`), and all other links connect through joints in a parent-child hierarchy. For a humanoid:

```
torso (root)
├── left_upper_arm (via left_shoulder joint)
│   └── left_forearm (via left_elbow joint)
│       └── left_hand (via left_wrist joint)
├── right_upper_arm (via right_shoulder joint)
│   └── right_forearm (via right_elbow joint)
│       └── right_hand (via right_wrist joint)
└── head (via neck joint)
```

This tree structure is crucial: when the torso moves, all connected limbs move with it. When you rotate the shoulder, the entire arm (forearm + hand) rotates because they're descendants in the tree. Motion planning algorithms use this tree to compute **forward kinematics** (given joint angles, where is the hand?) and **inverse kinematics** (to reach a point, what joint angles are needed?).

### Simple Example: Two-Link Robot Arm

Here's a complete minimal URDF for a simple robot arm with two links (upper arm and forearm) connected by an elbow joint:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Base link (fixed to world) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <!-- Upper arm link -->
  <link name="upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
    </visual>
  </link>

  <!-- Shoulder joint (connects base to upper arm) -->
  <joint name="shoulder" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- Forearm link -->
  <link name="forearm">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
    </visual>
  </link>

  <!-- Elbow joint (connects upper arm to forearm) -->
  <joint name="elbow" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="0" effort="10" velocity="1.0"/>
  </joint>

</robot>
```

This URDF defines a kinematic chain: base → shoulder joint → upper arm → elbow joint → forearm. Both joints are revolute (rotate around Y-axis), with the elbow limited to bending backward (negative angles).

### 💬 AI Colearning Prompt

> **Suggested Exploration**: Ask Claude or ChatGPT to explain the kinematic tree for a humanoid robot and how URDF represents it. Specifically: "For a humanoid robot with arms, legs, and a head, draw the kinematic tree showing parent-child relationships. Why does the torso typically serve as the root link? What happens to the hand's position when you rotate the shoulder joint?"
>
> Then extend: "In the simple arm example above, what would happen if you tried to make the forearm the parent and the upper arm the child? Why does the tree structure matter for motion planning?"
>
> This exploration will solidify your understanding of hierarchical robot models.

### 🎓 Expert Insight

**URDF Units and Coordinate Frame Conventions**

URDF has strict conventions that you must follow, or your robot will appear distorted in simulators and visualizers. Understanding these conventions prevents hours of debugging "why is my robot 1000x too big?" or "why is my arm pointing the wrong way?"

**Units Are Fixed**:
- **Distances**: Always meters (not centimeters, not millimeters). A 30cm forearm is `length="0.3"`, not `30`.
- **Angles**: Always radians (not degrees). A 90° rotation is `1.5708` (π/2), not `90`.
- **Mass**: Kilograms
- **Force/Torque**: Newtons and Newton-meters

Getting units wrong is the #1 beginner mistake. If your robot appears as a tiny speck or crashes through the floor in Gazebo, check your units first.

**Coordinate Frame Conventions (Right-Hand Rule)**:
- **X-axis**: Forward (typically the direction the robot faces)
- **Y-axis**: Left (perpendicular to forward)
- **Z-axis**: Up (vertical, against gravity)

Roll-Pitch-Yaw (rpy) rotations follow this convention:
- **Roll**: Rotation around X-axis (lean left/right)
- **Pitch**: Rotation around Y-axis (tilt forward/backward)
- **Yaw**: Rotation around Z-axis (turn left/right)

**Origin Placement Matters**: The `<origin>` tag in joints specifies where the joint is located **relative to the parent link's frame**. When debugging, visualize: "If I'm standing at the parent link's origin, looking along its X-axis, where is this joint?"

**For beginners in 2025**: Use RViz early and often. Load your URDF in RViz with `ros2 launch urdf_tutorial display.launch.py model:=your_robot.urdf` and immediately see if your link sizes, joint positions, and orientations are correct. Visual feedback catches unit and orientation errors that are hard to spot in XML.

## Practical Example

Let's model a realistic **humanoid robot arm** with shoulder, elbow, and wrist joints. This demonstrates how URDF represents a multi-joint kinematic chain with different joint types and realistic dimensions.

**The Scenario**: Your humanoid robot needs an arm for manipulation tasks. The arm attaches to the torso at the shoulder (3 degrees of freedom for reaching), bends at the elbow (1 DOF), and rotates at the wrist (1 DOF). We'll model this as a simplified URDF with appropriate link lengths and joint limits.

```xml
<?xml version="1.0"?>
<robot name="humanoid_arm">

  <!-- Torso (root link, fixed to world in this example) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.5 0.6"/>
      </geometry>
    </visual>
  </link>

  <!-- Upper arm link -->
  <link name="upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.28"/>
      </geometry>
      <origin xyz="0 0 -0.14" rpy="0 0 0"/>
    </visual>
  </link>

  <!-- Shoulder joint (revolute, rotates around Y-axis) -->
  <joint name="shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="upper_arm"/>
    <origin xyz="0 0.18 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
  </joint>

  <!-- Forearm link -->
  <link name="forearm">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
    </visual>
  </link>

  <!-- Elbow joint (revolute, bends arm) -->
  <joint name="elbow" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0 0 -0.28" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.356" effort="30" velocity="2.0"/>
  </joint>

  <!-- Hand link -->
  <link name="hand">
    <visual>
      <geometry>
        <box size="0.08 0.15 0.03"/>
      </geometry>
      <origin xyz="0.04 0 0" rpy="0 0 0"/>
    </visual>
  </link>

  <!-- Wrist joint (revolute, rotates hand) -->
  <joint name="wrist_roll" type="revolute">
    <parent link="forearm"/>
    <child link="hand"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="2.0"/>
  </joint>

</robot>
```

**Why This Design Works**:

- **Shoulder** at (0, 0.18, 0.25) positions the arm 18cm to the side and 25cm up from the torso's center
- **Upper arm** 28cm long with 5cm radius models a realistic humanoid arm segment
- **Elbow** connects at the upper arm's end (-0.28 Z offset) and allows 0 to 135° bending (0 to 2.356 rad)
- **Forearm** 25cm long, slightly thinner (4cm radius) than upper arm
- **Wrist** connects at forearm's end and rotates around Z-axis (hand pronation/supination)
- **Hand** is a simple box (8cm x 15cm x 3cm) representing a gripper placeholder

The kinematic tree is: torso → upper_arm → forearm → hand. When you rotate the shoulder, the entire arm moves. When you bend the elbow, only the forearm and hand move. This hierarchical dependency is how real robot arms work.

### 🤝 Practice Exercise

**Design a Humanoid Robot Leg**

Sketch a URDF structure for a humanoid robot leg with three joints: hip, knee, and ankle. Your leg should have four links (thigh, shin, foot, and a base link representing the pelvis).

**Your Task**:
1. Define realistic link dimensions (thigh ≈30cm, shin ≈35cm, foot ≈25cm long)
2. Create three revolute joints:
   - **Hip**: Connects pelvis to thigh, allows forward/backward swing (flexion/extension)
   - **Knee**: Connects thigh to shin, bends backward only (0 to 150°)
   - **Ankle**: Connects shin to foot, allows up/down (dorsiflexion/plantarflexion)
3. Choose appropriate joint axes (which direction does each joint rotate?)
4. Set realistic joint limits based on human anatomy

**Consider**:
- Which link should be the parent (root) of your leg?
- What happens if you set the knee's limit to allow forward bending (negative angles)?
- How would you add a second leg (hint: you'd need left and right versions of all links and joints)

Draw your URDF structure on paper or pseudocode, then ask an AI assistant to review your design. This exercise reinforces kinematic tree concepts and URDF joint definitions—skills you'll use in the capstone project.

## Summary

**Key Takeaways**:

- **URDF describes robot structure declaratively**: It's an XML format that specifies links (body parts) and joints (connections) without procedural code. Simulation, visualization, and planning tools read URDF to understand your robot's capabilities.

- **Links are rigid bodies, joints define motion**: Links represent physical components (arms, legs, sensors) with visual and collision geometry. Joints connect links and specify motion type (revolute, prismatic, fixed) with limits and axes.

- **Kinematic trees organize robots hierarchically**: Every URDF robot is a tree with a root link and parent-child relationships defined by joints. This tree structure enables forward/inverse kinematics and determines how moving one joint affects connected links.

- **URDF enables simulation and visualization**: Gazebo uses URDF for physics simulation (testing control algorithms safely). RViz uses URDF for 3D visualization (debugging robot behavior in real-time). MoveIt uses URDF for motion planning (computing collision-free trajectories).

- **Units and conventions are critical**: URDF uses meters (distance), radians (angles), and right-hand coordinate frames (X=forward, Y=left, Z=up). Getting units or axes wrong causes robots to appear distorted or behave incorrectly in simulation.

**What You Should Now Understand**: You can read a simple URDF file and identify links, joints, parent-child relationships, and joint types. You understand what URDF represents conceptually (robot structure as a kinematic tree) and why it's essential for simulation, visualization, and motion planning. You're ready to integrate all Module 1 concepts in the capstone project.

## Next Steps

You've completed all four foundational lessons in Module 1: The Robotic Nervous System (ROS 2). You learned **what ROS2 is** (Lesson 1), **how robot components communicate** (Lesson 2), **how to write robot code in Python** (Lesson 3), and **how to model robot structure** (Lesson 4).

In the **Capstone Project**, you'll integrate everything: design a multi-node ROS2 system for a humanoid robot, write Python rclpy code for publishers and subscribers, and create a URDF model that brings your design to life in simulation. This capstone demonstrates your mastery of the concepts and prepares you for advanced robotics topics like motion planning, computer vision integration, and autonomous behaviors.

With ROS2 fundamentals solid, you're ready to build intelligent robotic systems that perceive, plan, and act in the physical world. The nervous system is in place—now you'll teach it to think.
