"""
Test Selected Text Feature

Tests the chatbot's ability to explain selected text from course materials.
"""

import requests
import json

# Test the selected text feature
url = "http://127.0.0.1:8001/api/chat/query"

# Test Case 1: Selected text with question
selected_text_1 = """ROS2 (Robot Operating System 2) is an open-source middleware framework that serves as the communication backbone for modern robots. Think of it as the nervous system that allows different parts of a robot—sensors, motors, cameras, decision-making algorithms—to talk to each other efficiently and reliably."""

payload_1 = {
    "query": "Can you explain what ROS2 is in simpler terms?",
    "selected_text": selected_text_1,
    "conversation_history": []
}

# Test Case 2: Selected text asking for elaboration
selected_text_2 = """In ROS2, a node is an independent process responsible for a single, well-defined task within your robot system."""

payload_2 = {
    "query": "What are some examples of nodes in a real robot?",
    "selected_text": selected_text_2,
    "conversation_history": []
}

# Test Case 3: Too short selected text (should be ignored)
payload_3 = {
    "query": "What is ROS2?",
    "selected_text": "ROS2",  # Too short (< 10 chars)
    "conversation_history": []
}

# Test Case 4: No selected text (normal query)
payload_4 = {
    "query": "What is ROS2 and why is it important?",
    "conversation_history": []
}

test_cases = [
    ("Test 1: Selected text with simplification request", payload_1),
    ("Test 2: Selected text with examples request", payload_2),
    ("Test 3: Too short selected text (should be ignored)", payload_3),
    ("Test 4: Normal query without selected text", payload_4)
]

print("="*70)
print("SELECTED TEXT FEATURE TEST")
print("="*70)

for i, (test_name, payload) in enumerate(test_cases, 1):
    print(f"\n{'='*70}")
    print(f"{test_name}")
    print(f"{'='*70}")

    try:
        print(f"[INFO] Query: {payload['query']}")
        if payload.get('selected_text'):
            print(f"[INFO] Selected text: {payload['selected_text'][:100]}...")
        else:
            print(f"[INFO] No selected text")

        response = requests.post(url, json=payload, timeout=30)

        if response.status_code == 200:
            data = response.json()
            print(f"\n[SUCCESS] Response received!")
            print(f"\n{'─'*70}")
            print("CHATBOT RESPONSE:")
            print(f"{'─'*70}")
            print(data['response'])
            print(f"\n{'─'*70}")
            print(f"Sources: {len(data.get('sources', []))}")
            print(f"Retrieved chunks: {data.get('retrieved_chunks', 'N/A')}")
            print(f"Used selected text: {data.get('used_selected_text', False)}")
        else:
            print(f"[ERROR] Request failed with status {response.status_code}")
            print(response.text)

    except requests.exceptions.ConnectionError:
        print("[ERROR] Could not connect to API")
        print("Make sure the server is running: uvicorn app.main:app --port 8001")
        break
    except Exception as e:
        print(f"[ERROR] {e}")

    if i < len(test_cases):
        input("\nPress Enter to continue to next test...")

print(f"\n{'='*70}")
print("TEST COMPLETE")
print(f"{'='*70}")
