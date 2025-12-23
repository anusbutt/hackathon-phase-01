---
title: Lesson 2 - Cognitive Planning with LLMs
sidebar_position: 2
skills:
  - Large Language Models
  - Task Decomposition
  - Natural Language Understanding
  - Prompt Engineering
  - ROS2 Action Planning
learning_objectives:
  - "Understand how LLMs decompose high-level natural language commands into action sequences"
  - "Learn effective prompting strategies for robotics task planning"
  - "Implement task decomposition algorithms for natural language to ROS2 actions"
  - "Handle ambiguous or underspecified commands with LLMs"
  - "Design error recovery strategies for LLM-generated action sequences"
cognitive_load: 6
differentiation: "AI Colearning, Expert Insight, Practice Exercise"
tags:
  - llm
  - cognitive-planning
  - task-decomposition
  - robotics
  - natural-language
created: "2025-12-23"
last_modified: "2025-12-23"
ros2_version: humble
---

# Lesson 2: Cognitive Planning with LLMs

## What Is Cognitive Planning?

Cognitive planning represents the AI brain of modern robotic systems, where Large Language Models (LLMs) serve as the reasoning engine that translates high-level human intentions into executable robotic actions. Unlike traditional programming approaches that require explicit step-by-step instructions, cognitive planning uses LLMs to understand natural language commands and automatically decompose them into sequences of specific robot behaviors.

At its core, cognitive planning involves several key components: natural language understanding to interpret the command, task decomposition to break complex goals into simpler subtasks, spatial reasoning to understand the environment and object relationships, and action mapping to connect abstract concepts to concrete ROS2 actions. This process mimics human cognitive planning, where we naturally decompose complex goals into manageable steps.

The LLM acts as an intelligent intermediary between human intent and robot execution. When given a command like "Clean the room," the LLM must understand that this involves multiple subtasks: identifying what constitutes "cleaning," determining what areas need attention, recognizing objects that might need to be moved or disposed of, and sequencing these actions appropriately. This requires not just language understanding, but also commonsense reasoning about the physical world.

Modern cognitive planning systems often employ techniques like Chain-of-Thought (CoT) prompting, where the LLM is guided to think through the problem step-by-step before providing the final action sequence. This approach improves the reliability and interpretability of the planning process, making it easier to debug when the robot behaves unexpectedly.

## Why Cognitive Planning Matters

Cognitive planning bridges the critical gap between human communication and robot execution, enabling complex autonomous behaviors from simple natural language commands. Without this capability, robots would require explicit programming for every possible task, making them inflexible and difficult to deploy in dynamic environments where new tasks emerge regularly.

For humanoid robots specifically, cognitive planning enables true collaboration with humans. Instead of requiring users to learn robot-specific commands or programming languages, cognitive planning allows natural communication. A user can say "Please set the table for dinner" and the robot can decompose this into: identifying table location, determining appropriate place settings, retrieving plates, utensils, and glasses, and arranging them appropriately.

The scalability aspect is crucial for practical robotics deployment. Traditional programming approaches require an exponential increase in complexity as new tasks are added. Cognitive planning, however, can generalize from existing knowledge to handle novel commands that follow similar patterns. This makes robots more adaptable and cost-effective to deploy.

Furthermore, cognitive planning enables robots to handle ambiguous or underspecified commands gracefully. When a user says "Clean up over there," the robot can ask clarifying questions or make reasonable assumptions based on context, rather than failing completely as a traditional programmed system might.

## Key Principles

### Chain-of-Thought Prompting
This technique guides LLMs to reason step-by-step before providing the final answer. For robotics, this might involve asking the LLM to first identify the goal, then list required subtasks, consider constraints, and finally provide the action sequence. This approach improves reliability and provides interpretable reasoning paths.

### Task Decomposition Hierarchies
Complex tasks are broken down into hierarchies of subtasks. For example, "Clean the room" decomposes into "Identify dirty items," "Categorize items," "Dispose of trash," "Put items in proper places," and so on. Each subtask can be further decomposed until reaching primitive actions executable by the robot.

### World Modeling and Context Awareness
Effective cognitive planning requires the LLM to maintain a model of the world state and how actions affect it. This includes understanding object affordances (what can be done with an object), spatial relationships, and the effects of actions on the environment.

### Grounding in Physical Reality
LLM outputs must be grounded in the physical capabilities and constraints of the robot. A plan that works in simulation or abstract reasoning must be feasible given the robot's mobility, manipulation capabilities, and sensor limitations.

### Error Handling and Recovery
Cognitive planning systems must handle cases where LLM-generated plans fail or where the physical world differs from the LLM's assumptions. This includes fallback strategies, clarification requests, and graceful degradation when perfect execution isn't possible.

## 💬 AI Colearning Prompt
Ask Claude to demonstrate how "Clean the room" gets decomposed into robotic actions. Consider the intermediate reasoning steps that would be important for a cognitive planning system to consider.

## 🎓 Expert Insight
LLMs have inherent limitations when applied to robotics planning. They can hallucinate information not present in their training data, struggle with precise spatial reasoning, and may generate plans that seem reasonable but are physically impossible. Robust cognitive planning systems must include validation mechanisms to catch these issues before execution. Additionally, LLMs may generate different responses to identical prompts due to their probabilistic nature, requiring careful consideration of consistency in safety-critical applications.

## Practical Example: LLM-Based Task Planning for Cleaning

Consider a humanoid robot receiving the command "Clean the living room." A cognitive planning system would process this command through several stages:

First, the LLM interprets the command and identifies the key components: the goal (cleaning), the location (living room), and the implied constraints (don't damage furniture, return items to proper places). The system might use a prompt template like: "Decompose the task 'Clean the living room' into specific robotic actions. Consider what cleaning involves, what objects are typically found in a living room, and what actions are needed to make the space clean."

The LLM generates a hierarchical plan:
1. Navigate to living room
2. Survey environment to identify objects and obstacles
3. Categorize objects (trash, misplaced items, furniture to be preserved)
4. For each category:
   - Trash: Pick up and dispose
   - Misplaced items: Identify correct location and return
   - Furniture: Leave in place
5. Return to home position

The cognitive planning system then maps these high-level steps to specific ROS2 actions. "Navigate to living room" becomes a Navigation2 goal. "Survey environment" triggers perception nodes. "Pick up and dispose" decomposes into manipulation action sequences.

Throughout this process, the system maintains awareness of constraints: the robot cannot lift objects heavier than its payload capacity, it must avoid collisions, and it should preserve valuable items. If the robot encounters an unexpected obstacle during navigation, the system can replan or request assistance.

This example demonstrates how cognitive planning transforms abstract human goals into concrete robot behaviors while maintaining flexibility to handle real-world complexities.

## 🤝 Practice Exercise
Analyze the command "Set up for a meeting" and break it down into ROS2 action sequences. Consider what objects might be needed (projector, chairs, whiteboard markers), where they might be located, and in what order the setup should occur. What clarifying questions might the robot need to ask?

## Summary

Cognitive planning with LLMs represents a paradigm shift in robotics, enabling robots to understand and execute complex tasks from natural language commands. The key components include natural language understanding, task decomposition, world modeling, and action mapping. Effective systems employ techniques like Chain-of-Thought prompting and hierarchical task decomposition to generate reliable plans.

The approach enables unprecedented flexibility and natural human-robot interaction, allowing robots to handle novel tasks without explicit programming. However, successful implementation requires careful attention to grounding in physical reality, error handling, and validation of LLM outputs.

As LLM capabilities continue to advance, cognitive planning will become increasingly sophisticated, enabling robots to handle more complex and nuanced tasks. The integration of cognitive planning with other robotic capabilities like perception and action execution creates truly autonomous systems capable of meaningful collaboration with humans.

## Next Steps

In the next lesson, we'll explore how computer vision and language models work together in Vision-Language integration systems. We'll examine how robots identify and manipulate objects mentioned in natural language commands, building on the cognitive planning foundation we've established here.