"""
Add Test Data to Qdrant

Manually adds a few test chunks with mock embeddings to Qdrant
so we can develop and test the RAG chatbot without Cohere embeddings.

Usage:
    python scripts/add_test_data.py
"""

import sys
import os
from pathlib import Path
import uuid
import random

sys.path.insert(0, str(Path(__file__).parent.parent))

from dotenv import load_dotenv
from app.services.qdrant_service import QdrantService

load_dotenv()


def generate_mock_embedding(dim=1024):
    """Generate a random embedding vector (for testing only)."""
    return [random.uniform(-1, 1) for _ in range(dim)]


def main():
    print("[INFO] Adding test data to Qdrant...")

    # Initialize Qdrant service
    qdrant_url = os.getenv('QDRANT_URL')
    qdrant_api_key = os.getenv('QDRANT_API_KEY')
    qdrant_collection = os.getenv('QDRANT_COLLECTION_NAME', 'humanoid_robotics_book')

    qdrant_service = QdrantService(qdrant_url, qdrant_api_key, qdrant_collection)

    # Test chunks
    test_chunks = [
        {
            'content': """## What Is ROS2?

ROS2 (Robot Operating System 2) is an open-source middleware framework that serves as the communication backbone for modern robots. Think of it as the nervous system that allows different parts of a robot—sensors, motors, cameras, decision-making algorithms—to talk to each other efficiently and reliably.

Unlike traditional software where all code runs in a single program, ROS2 enables distributed systems architecture. This means your robot's vision processing can run on one computer, motion planning on another, and motor control on embedded hardware, all communicating seamlessly.""",
            'metadata': {
                'module': '01-ros2-nervous-system',
                'lesson': '01-ros2-fundamentals',
                'lesson_title': 'ROS2 Fundamentals',
                'module_title': 'Module 1: ROS2 Nervous System',
                'section': 'What Is ROS2?',
                'tags': ['ros2', 'middleware', 'fundamentals'],
                'skills': ['ROS2 Architecture Understanding'],
                'bloom_level': 'understand'
            }
        },
        {
            'content': """## What Are Nodes?

In ROS2, a node is an independent process responsible for a single, well-defined task within your robot system. Think of nodes as specialized workers in a factory: one worker handles packaging, another handles quality control, another manages inventory. Each node focuses on doing one thing well, and they coordinate through structured communication.

For example, a humanoid robot might have a camera_node that captures images, an object_detection_node that identifies objects, a path_planning_node that decides where to move, and a motor_control_node that executes movements.""",
            'metadata': {
                'module': '01-ros2-nervous-system',
                'lesson': '02-nodes-topics-services',
                'lesson_title': 'Nodes, Topics, and Services',
                'module_title': 'Module 1: ROS2 Nervous System',
                'section': 'What Are Nodes?',
                'tags': ['ros2', 'nodes', 'architecture'],
                'skills': ['Node Communication Patterns'],
                'bloom_level': 'understand'
            }
        },
        {
            'content': """## What Is rclpy?

rclpy (ROS Client Library for Python) is the official Python API for ROS2. It's the bridge that connects your Python programming skills to the powerful ROS2 middleware infrastructure. When you write robot code in Python, rclpy handles all the complex networking, message serialization, and inter-process communication behind the scenes.

Think of rclpy as a translator. You speak Python—a high-level, readable language you already know. ROS2 speaks DDS—a low-level, high-performance networking protocol. rclpy translates your Python instructions into efficient ROS2 operations.""",
            'metadata': {
                'module': '01-ros2-nervous-system',
                'lesson': '03-python-rclpy-bridge',
                'lesson_title': 'Python rclpy Bridge',
                'module_title': 'Module 1: ROS2 Nervous System',
                'section': 'What Is rclpy?',
                'tags': ['ros2', 'python', 'rclpy', 'api'],
                'skills': ['Python ROS2 Programming'],
                'bloom_level': 'understand'
            }
        },
        {
            'content': """## Why ROS2 Matters for Physical AI

Building a humanoid robot is fundamentally different from writing a web application or training a machine learning model. A humanoid must walk, maintain balance, perceive its environment, manipulate objects, and make decisions—all simultaneously and in real-time. No single computer can handle this complexity efficiently. This is where ROS2 becomes essential.

The Modularity Problem: Imagine trying to build a humanoid robot where all the code lives in one giant program. Vision processing, path planning, motor control, speech recognition—everything tangled together.""",
            'metadata': {
                'module': '01-ros2-nervous-system',
                'lesson': '01-ros2-fundamentals',
                'lesson_title': 'ROS2 Fundamentals',
                'module_title': 'Module 1: ROS2 Nervous System',
                'section': 'Why ROS2 Matters for Physical AI',
                'tags': ['ros2', 'physical-ai', 'architecture', 'distributed-systems'],
                'skills': ['Distributed Systems Concepts'],
                'bloom_level': 'understand'
            }
        },
        {
            'content': """## Sensors in Humanoid Robotics

Sensors are the sensory organs of a humanoid robot—they provide the perception data that enables intelligent decision-making and responsive behavior. Just as humans use sight, touch, hearing, and proprioception to navigate the world, humanoid robots rely on cameras, IMUs, force sensors, and encoders to understand their environment and their own body state.

In Physical AI systems, sensors feed raw data into perception pipelines that extract meaningful information: "There's a door handle at waist height" from camera images, "My left foot just made contact with the ground" from force sensors.""",
            'metadata': {
                'module': '02-sensors-perception',
                'lesson': '01-sensor-fundamentals',
                'lesson_title': 'Sensor Fundamentals',
                'module_title': 'Module 2: Sensors & Perception',
                'section': 'Sensors in Humanoid Robotics',
                'tags': ['sensors', 'perception', 'hardware'],
                'skills': ['Sensor Integration'],
                'bloom_level': 'understand'
            }
        }
    ]

    # Upload to Qdrant
    points = []
    for chunk in test_chunks:
        chunk_id = str(uuid.uuid4())

        payload = {
            'chunk_id': chunk_id,
            'content': chunk['content'],
            **chunk['metadata']
        }

        points.append({
            'id': chunk_id,
            'vector': generate_mock_embedding(1024),
            'payload': payload
        })

    qdrant_service.client.upsert(
        collection_name=qdrant_service.collection_name,
        points=points
    )

    print(f"[SUCCESS] Added {len(points)} test chunks to Qdrant")
    print(f"Collection: {qdrant_collection}")
    print("\nTest chunks:")
    for i, chunk in enumerate(test_chunks, 1):
        print(f"  {i}. {chunk['metadata']['lesson_title']} - {chunk['metadata']['section']}")


if __name__ == '__main__':
    main()
