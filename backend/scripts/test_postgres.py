"""
Neon Postgres Connection Test Script

Tests connection to Neon Postgres and verifies tables work.

Usage:
    python scripts/test_postgres.py
"""

import sys
import os
from pathlib import Path
import asyncio

sys.path.insert(0, str(Path(__file__).parent.parent))

import asyncpg
from dotenv import load_dotenv
import uuid

load_dotenv()


async def test_postgres():
    """Test Postgres connection and basic operations."""

    database_url = os.getenv("NEON_DATABASE_URL")

    if not database_url:
        print("❌ Error: NEON_DATABASE_URL must be set in .env file")
        sys.exit(1)

    try:
        # Connect
        print(f"🔌 Connecting to Neon Postgres...")
        conn = await asyncpg.connect(database_url)

        # Test users table
        print(f"🧪 Testing 'users' table...")
        test_user_id = await conn.fetchval("""
            INSERT INTO users (username, email)
            VALUES ($1, $2)
            RETURNING id
        """, f"test_user_{uuid.uuid4().hex[:8]}", f"test_{uuid.uuid4().hex[:8]}@example.com")
        print(f"✅ Test user created (ID: {test_user_id})")

        # Retrieve user
        user = await conn.fetchrow("""
            SELECT username, email, created_at
            FROM users
            WHERE id = $1
        """, test_user_id)
        print(f"✅ Test user retrieved: {user['username']}")

        # Test conversations table
        print(f"🧪 Testing 'conversations' table...")
        test_conv_id = await conn.fetchval("""
            INSERT INTO conversations (user_id, query, response)
            VALUES ($1, $2, $3)
            RETURNING id
        """, test_user_id, "Test query?", "Test response.")
        print(f"✅ Test conversation created (ID: {test_conv_id})")

        # Test user_progress table
        print(f"🧪 Testing 'user_progress' table...")
        await conn.execute("""
            INSERT INTO user_progress (user_id, module, lesson, completed)
            VALUES ($1, $2, $3, $4)
        """, test_user_id, "module-01", "lesson-01", True)
        print(f"✅ Test progress record created")

        # Clean up test data
        print(f"🗑️  Cleaning up test data...")
        await conn.execute("DELETE FROM user_progress WHERE user_id = $1", test_user_id)
        await conn.execute("DELETE FROM conversations WHERE user_id = $1", test_user_id)
        await conn.execute("DELETE FROM users WHERE id = $1", test_user_id)
        print(f"✅ Test data deleted")

        await conn.close()
        print("\n✨ All Postgres tests passed! Connection verified.")

    except Exception as e:
        print(f"❌ Error testing Postgres: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    print("=" * 60)
    print("Neon Postgres Connection Test")
    print("=" * 60)
    asyncio.run(test_postgres())
