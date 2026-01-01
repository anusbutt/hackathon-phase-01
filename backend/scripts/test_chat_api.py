"""
Test Chat API

Simple script to test the chat endpoint with a sample query.
"""

import requests
import json

# Test the chat endpoint
url = "http://127.0.0.1:8001/api/chat/query"

payload = {
    "query": "What is ROS2 and why is it important for robotics?",
    "conversation_history": []
}

try:
    print("[INFO] Sending test query to chat API...")
    print(f"[INFO] Query: {payload['query']}\n")

    response = requests.post(url, json=payload, timeout=30)

    if response.status_code == 200:
        data = response.json()
        print("[SUCCESS] Response received!\n")
        print("="*60)
        print("CHATBOT RESPONSE:")
        print("="*60)
        print(data['response'])
        print("\n" + "="*60)
        print("SOURCES:")
        print("="*60)
        for source in data['sources']:
            section = f" > {source['section']}" if source.get('section') else ""
            print(f"- {source['module']} > {source['lesson']}{section}")
        print("\n" + "="*60)
        print(f"Conversation ID: {data['conversation_id']}")
        print(f"Retrieved chunks: {data.get('retrieved_chunks', 'N/A')}")
    else:
        print(f"[ERROR] Request failed with status {response.status_code}")
        print(response.text)

except requests.exceptions.ConnectionError:
    print("[ERROR] Could not connect to API")
    print("Make sure the server is running: uvicorn app.main:app --port 8001")
except Exception as e:
    print(f"[ERROR] {e}")
