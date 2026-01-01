"""
Qdrant Cloud Setup Script

Creates the Qdrant collection for storing book content embeddings.
Run this script once after creating your Qdrant Cloud account.

Usage:
    python scripts/setup_qdrant.py
"""

import sys
import os
from pathlib import Path

# Add parent directory to path to import app modules
sys.path.insert(0, str(Path(__file__).parent.parent))

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams
from dotenv import load_dotenv

# Load environment variables
load_dotenv()


def setup_qdrant():
    """Create Qdrant collection for book content."""

    # Get configuration from environment
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "humanoid_robotics_book")

    if not qdrant_url or not qdrant_api_key:
        print("[ERROR] QDRANT_URL and QDRANT_API_KEY must be set in .env file")
        print("\nPlease:")
        print("1. Create Qdrant Cloud account at https://cloud.qdrant.io/")
        print("2. Create a cluster (free tier)")
        print("3. Get your API key and cluster URL")
        print("4. Add to .env file")
        sys.exit(1)

    try:
        # Initialize Qdrant client
        print(f"[INFO] Connecting to Qdrant Cloud at {qdrant_url}...")
        client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key
        )

        # Check if collection already exists
        collections = client.get_collections().collections
        collection_names = [col.name for col in collections]

        if collection_name in collection_names:
            print(f"[WARNING] Collection '{collection_name}' already exists")
            response = input("Do you want to recreate it? (yes/no): ")
            if response.lower() == "yes":
                print(f"[INFO] Deleting existing collection...")
                client.delete_collection(collection_name)
            else:
                print("[SUCCESS] Using existing collection")
                return

        # Create collection
        print(f"[INFO] Creating collection '{collection_name}'...")
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=1024,  # Cohere embed-english-v3.0 dimension
                distance=Distance.COSINE
            )
        )

        # Verify collection was created
        info = client.get_collection(collection_name)
        print(f"[SUCCESS] Collection created successfully!")
        print(f"   - Name: {collection_name}")
        print(f"   - Vector size: {info.config.params.vectors.size}")
        print(f"   - Distance: {info.config.params.vectors.distance}")
        print(f"   - Points: {info.points_count}")

        print("\n[SUCCESS] Qdrant setup complete! Ready for embedding pipeline.")

    except Exception as e:
        print(f"[ERROR] Error setting up Qdrant: {e}")
        sys.exit(1)


if __name__ == "__main__":
    print("=" * 60)
    print("Qdrant Cloud Setup for RAG Chatbot")
    print("=" * 60)
    setup_qdrant()
