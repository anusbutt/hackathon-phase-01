"""
Neon Postgres Schema Setup Script

Creates database tables for user data and conversations.
Run this script once after creating your Neon Postgres database.

Usage:
    python scripts/setup_postgres.py
"""

import sys
import os
from pathlib import Path
import asyncio

sys.path.insert(0, str(Path(__file__).parent.parent))

import asyncpg
from dotenv import load_dotenv

load_dotenv()


async def setup_postgres():
    """Create database schema for RAG chatbot."""

    database_url = os.getenv("NEON_DATABASE_URL")

    if not database_url:
        print("[ERROR] NEON_DATABASE_URL must be set in .env file")
        print("\nPlease:")
        print("1. Create Neon Postgres account at https://neon.tech/")
        print("2. Create a project (free tier)")
        print("3. Get your connection string")
        print("4. Add to .env file")
        sys.exit(1)

    try:
        # Connect to database
        print(f"[INFO] Connecting to Neon Postgres...")
        conn = await asyncpg.connect(database_url)

        # Create users table
        print(f"[INFO] Creating 'users' table...")
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS users (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                username VARCHAR(255) UNIQUE,
                email VARCHAR(255) UNIQUE,
                created_at TIMESTAMP DEFAULT NOW(),
                current_module VARCHAR(100),
                current_lesson VARCHAR(100),
                preferences JSONB DEFAULT '{}'::jsonb
            );
        """)
        print(f"[SUCCESS] 'users' table created")

        # Create conversations table
        print(f"[INFO] Creating 'conversations' table...")
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS conversations (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                user_id UUID REFERENCES users(id),
                query TEXT NOT NULL,
                response TEXT NOT NULL,
                selected_text TEXT,
                lesson_id VARCHAR(255),
                sources JSONB,
                created_at TIMESTAMP DEFAULT NOW(),
                rating INTEGER CHECK (rating >= 1 AND rating <= 5)
            );
        """)
        print(f"[SUCCESS] 'conversations' table created")

        # Create user_progress table
        print(f"[INFO] Creating 'user_progress' table...")
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS user_progress (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                user_id UUID REFERENCES users(id),
                module VARCHAR(100),
                lesson VARCHAR(100),
                completed BOOLEAN DEFAULT FALSE,
                time_spent_seconds INTEGER DEFAULT 0,
                quiz_score INTEGER,
                last_accessed TIMESTAMP DEFAULT NOW(),
                UNIQUE(user_id, module, lesson)
            );
        """)
        print(f"[SUCCESS] 'user_progress' table created")

        # Create indexes
        print(f"[INFO] Creating indexes...")
        await conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_conversations_user_id
            ON conversations(user_id);
        """)
        await conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_conversations_created_at
            ON conversations(created_at DESC);
        """)
        await conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_user_progress_user_id
            ON user_progress(user_id);
        """)
        print(f"[SUCCESS] Indexes created")

        # Verify tables
        tables = await conn.fetch("""
            SELECT table_name
            FROM information_schema.tables
            WHERE table_schema = 'public'
            AND table_name IN ('users', 'conversations', 'user_progress');
        """)

        print(f"\n[INFO] Created tables:")
        for table in tables:
            print(f"   - {table['table_name']}")

        await conn.close()
        print("\n[SUCCESS] Postgres schema setup complete!")

    except Exception as e:
        print(f"[ERROR] Error setting up Postgres: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    print("=" * 60)
    print("Neon Postgres Schema Setup for RAG Chatbot")
    print("=" * 60)
    asyncio.run(setup_postgres())
