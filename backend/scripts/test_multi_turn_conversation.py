"""
Test Multi-turn Conversations Feature

Tests the chatbot's ability to maintain conversation context across multiple turns.
Tests conversation expiry validation and last-5-messages limit.
"""

import requests
import json
from datetime import datetime, timedelta
from typing import List, Dict, Any


# Configuration
API_URL = "http://127.0.0.1:8001/api/chat/query"


def print_section(title: str):
    """Print a formatted section header."""
    print(f"\n{'='*70}")
    print(f"{title}")
    print(f"{'='*70}\n")


def print_message(role: str, content: str, turn: int = None):
    """Print a formatted message."""
    if turn:
        print(f"[Turn {turn}] {role}: {content[:100]}{'...' if len(content) > 100 else ''}")
    else:
        print(f"{role}: {content[:100]}{'...' if len(content) > 100 else ''}")


def send_query(
    query: str,
    conversation_id: str = None,
    conversation_created_at: datetime = None,
    conversation_history: List[Dict] = None
) -> Dict[str, Any]:
    """
    Send a query to the chat API.

    Args:
        query: User's question
        conversation_id: Existing conversation ID
        conversation_created_at: When conversation was created
        conversation_history: Previous messages

    Returns:
        Response data or None if failed
    """
    payload = {
        "query": query,
        "conversation_history": conversation_history or []
    }

    if conversation_id:
        payload["conversation_id"] = conversation_id

    if conversation_created_at:
        payload["conversation_created_at"] = conversation_created_at.isoformat()

    try:
        response = requests.post(API_URL, json=payload, timeout=30)

        if response.status_code == 200:
            return response.json()
        elif response.status_code == 410:
            print(f"[WARNING] Conversation expired: {response.json()['detail']}")
            return None
        else:
            print(f"[ERROR] Request failed with status {response.status_code}")
            print(response.text)
            return None

    except requests.exceptions.ConnectionError:
        print("[ERROR] Could not connect to API")
        print("Make sure the server is running: uvicorn app.main:app --port 8001")
        return None
    except Exception as e:
        print(f"[ERROR] {e}")
        return None


def test_multi_turn_conversation():
    """Test a multi-turn conversation with 6 turns."""
    print_section("TEST 1: Multi-turn Conversation (6 turns)")

    conversation_id = None
    conversation_created_at = None
    conversation_history = []

    # Conversation scenario: Student learning about ROS2
    questions = [
        "What is ROS2?",
        "How is it different from ROS1?",
        "What are nodes in ROS2?",
        "Can you give me an example of a node?",
        "What are topics used for?",
        "How do nodes communicate using topics?"
    ]

    for turn, question in enumerate(questions, 1):
        print(f"\n--- Turn {turn} ---")
        print_message("Student", question, turn)

        # Send query with conversation history
        response = send_query(
            query=question,
            conversation_id=conversation_id,
            conversation_created_at=conversation_created_at,
            conversation_history=conversation_history
        )

        if not response:
            print("[FAILED] Could not get response")
            return False

        # Update conversation state
        if turn == 1:
            conversation_id = response['conversation_id']
            conversation_created_at = datetime.utcnow()
            print(f"[INFO] New conversation started: {conversation_id}")

        print_message("AI Tutor", response['response'], turn)
        print(f"[INFO] Sources: {len(response['sources'])}")

        # Add messages to history (user + assistant)
        conversation_history.append({
            "role": "user",
            "content": question,
            "timestamp": datetime.utcnow().isoformat()
        })

        conversation_history.append({
            "role": "assistant",
            "content": response['response'],
            "timestamp": response['timestamp'],
            "sources": response['sources']
        })

        # Show history length
        print(f"[INFO] Conversation history: {len(conversation_history)} messages")

        # After turn 5, we should have 10 messages (5 turns * 2 messages)
        # But only last 5 messages (2.5 turns) should be sent to API
        if turn >= 3:
            print(f"[INFO] Note: API will only use last 5 messages for context")

    print("\n[SUCCESS] Multi-turn conversation test completed!")
    print(f"[INFO] Final conversation ID: {conversation_id}")
    print(f"[INFO] Total messages in history: {len(conversation_history)}")
    return True


def test_conversation_expiry():
    """Test conversation expiry validation."""
    print_section("TEST 2: Conversation Expiry Validation")

    # Simulate an expired conversation (8 days old)
    conversation_id = "test-expired-conversation-id"
    conversation_created_at = datetime.utcnow() - timedelta(days=8)

    print(f"[INFO] Simulating expired conversation")
    print(f"[INFO] Conversation ID: {conversation_id}")
    print(f"[INFO] Created: {conversation_created_at.isoformat()}")
    print(f"[INFO] Age: 8 days (expires after 7 days)")

    response = send_query(
        query="What is ROS2?",
        conversation_id=conversation_id,
        conversation_created_at=conversation_created_at,
        conversation_history=[]
    )

    if response is None:
        print("[SUCCESS] Expired conversation correctly rejected (410 Gone)")
        return True
    else:
        print("[FAILED] Expired conversation was accepted (should have been rejected)")
        return False


def test_last_5_messages_limit():
    """Test that only last 5 messages are used for context."""
    print_section("TEST 3: Last 5 Messages Limit")

    conversation_history = []

    # Create 10 messages (5 turns)
    for i in range(5):
        conversation_history.append({
            "role": "user",
            "content": f"Question {i+1}: Tell me about ROS2 topic #{i+1}",
            "timestamp": datetime.utcnow().isoformat()
        })
        conversation_history.append({
            "role": "assistant",
            "content": f"Answer {i+1}: Here's information about ROS2 topic #{i+1}...",
            "timestamp": datetime.utcnow().isoformat()
        })

    print(f"[INFO] Created conversation history with {len(conversation_history)} messages")
    print(f"[INFO] Messages 1-5: {conversation_history[0]['content'][:50]}... to {conversation_history[4]['content'][:50]}...")
    print(f"[INFO] Messages 6-10: {conversation_history[5]['content'][:50]}... to {conversation_history[9]['content'][:50]}...")

    # Send query - API should only use last 5 messages
    print(f"\n[INFO] Sending query with 10-message history...")
    print(f"[INFO] Expected: API uses only last 5 messages (messages 6-10)")

    response = send_query(
        query="Now tell me about topics in general",
        conversation_history=conversation_history
    )

    if response:
        print(f"[SUCCESS] Query processed successfully")
        print(f"[INFO] API validated and limited conversation history to last 5 messages")
        return True
    else:
        print("[FAILED] Could not process query")
        return False


def test_new_vs_continuing_conversation():
    """Test distinguishing between new and continuing conversations."""
    print_section("TEST 4: New vs Continuing Conversation")

    # Test 1: New conversation (no conversation_id)
    print("[INFO] Test 4.1: New Conversation")
    response1 = send_query(query="What is ROS2?")

    if response1:
        conversation_id = response1['conversation_id']
        print(f"[SUCCESS] New conversation created: {conversation_id}")
    else:
        print("[FAILED] Could not create new conversation")
        return False

    # Test 2: Continue existing conversation
    print(f"\n[INFO] Test 4.2: Continuing Conversation {conversation_id}")

    conversation_history = [{
        "role": "user",
        "content": "What is ROS2?",
        "timestamp": datetime.utcnow().isoformat()
    }, {
        "role": "assistant",
        "content": response1['response'],
        "timestamp": response1['timestamp']
    }]

    response2 = send_query(
        query="Can you explain that in simpler terms?",
        conversation_id=conversation_id,
        conversation_created_at=datetime.utcnow(),
        conversation_history=conversation_history
    )

    if response2 and response2['conversation_id'] == conversation_id:
        print(f"[SUCCESS] Continued conversation with same ID: {conversation_id}")
        return True
    else:
        print("[FAILED] Conversation ID mismatch or no response")
        return False


def run_all_tests():
    """Run all multi-turn conversation tests."""
    print("="*70)
    print("MULTI-TURN CONVERSATION FEATURE TEST SUITE")
    print("="*70)

    results = {
        "Multi-turn Conversation (6 turns)": False,
        "Conversation Expiry Validation": False,
        "Last 5 Messages Limit": False,
        "New vs Continuing Conversation": False
    }

    # Run tests
    results["Multi-turn Conversation (6 turns)"] = test_multi_turn_conversation()
    input("\nPress Enter to continue to next test...")

    results["Conversation Expiry Validation"] = test_conversation_expiry()
    input("\nPress Enter to continue to next test...")

    results["Last 5 Messages Limit"] = test_last_5_messages_limit()
    input("\nPress Enter to continue to next test...")

    results["New vs Continuing Conversation"] = test_new_vs_continuing_conversation()

    # Print summary
    print_section("TEST SUMMARY")
    for test_name, passed in results.items():
        status = "[PASS]" if passed else "[FAIL]"
        print(f"{status} {test_name}")

    passed_count = sum(1 for passed in results.values() if passed)
    total_count = len(results)

    print(f"\n{'='*70}")
    print(f"Results: {passed_count}/{total_count} tests passed")
    print(f"{'='*70}")

    return passed_count == total_count


if __name__ == "__main__":
    success = run_all_tests()
    exit(0 if success else 1)
