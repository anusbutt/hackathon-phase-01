"""
Qdrant Connection Test Script

Tests connection to Qdrant Cloud and verifies the collection works.

Usage:
    python scripts/test_qdrant.py
"""

import sys
import os
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct
from dotenv import load_dotenv
import uuid

load_dotenv()


def test_qdrant():
    """Test Qdrant connection and basic operations."""

    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "humanoid_robotics_book")

    if not qdrant_url or not qdrant_api_key:
        print("❌ Error: QDRANT_URL and QDRANT_API_KEY must be set in .env file")
        sys.exit(1)

    try:
        # Connect
        print(f"🔌 Connecting to Qdrant Cloud...")
        client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

        # Verify collection exists
        print(f"📦 Checking collection '{collection_name}'...")
        info = client.get_collection(collection_name)
        print(f"✅ Collection found: {info.points_count} points")

        # Insert test vector
        print(f"🧪 Inserting test vector...")
        test_id = str(uuid.uuid4())
        test_vector = [0.1] * 1024  # 1024-dimensional test vector

        client.upsert(
            collection_name=collection_name,
            points=[
                PointStruct(
                    id=test_id,
                    vector=test_vector,
                    payload={
                        "content": "Test content",
                        "module": "test-module",
                        "lesson": "test-lesson",
                        "section": "Test Section"
                    }
                )
            ]
        )
        print(f"✅ Test vector inserted (ID: {test_id})")

        # Search for test vector
        print(f"🔍 Searching for test vector...")
        results = client.search(
            collection_name=collection_name,
            query_vector=test_vector,
            limit=1
        )

        if len(results) > 0 and str(results[0].id) == test_id:
            print(f"✅ Test vector found! Score: {results[0].score}")
        else:
            print(f"⚠️  Test vector not found in search results")

        # Clean up test data
        print(f"🗑️  Cleaning up test data...")
        client.delete(
            collection_name=collection_name,
            points_selector=[test_id]
        )
        print(f"✅ Test data deleted")

        print("\n✨ All Qdrant tests passed! Connection verified.")

    except Exception as e:
        print(f"❌ Error testing Qdrant: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    print("=" * 60)
    print("Qdrant Connection Test")
    print("=" * 60)
    test_qdrant()
