"""
Postgres Service - Database Connection Manager

Handles all interactions with Neon Serverless Postgres.
"""

import asyncpg
from typing import Optional, Any, List
from contextlib import asynccontextmanager
import logging

logger = logging.getLogger(__name__)


class PostgresService:
    """
    Service class for Neon Postgres database operations.

    Provides connection pooling and query execution methods.
    """

    def __init__(self, database_url: str):
        """
        Initialize Postgres service.

        Args:
            database_url: PostgreSQL connection string
        """
        self.database_url = database_url
        self.pool: Optional[asyncpg.Pool] = None
        logger.info("Postgres service initialized")

    async def create_pool(self, min_size: int = 2, max_size: int = 10):
        """
        Create connection pool.

        Args:
            min_size: Minimum number of connections
            max_size: Maximum number of connections
        """
        try:
            self.pool = await asyncpg.create_pool(
                self.database_url,
                min_size=min_size,
                max_size=max_size
            )
            logger.info(f"Postgres connection pool created (min={min_size}, max={max_size})")
        except Exception as e:
            logger.error(f"Postgres pool creation error: {e}")
            raise

    async def close_pool(self):
        """Close connection pool."""
        if self.pool:
            await self.pool.close()
            logger.info("Postgres connection pool closed")

    @asynccontextmanager
    async def get_connection(self):
        """
        Get database connection from pool (context manager).

        Usage:
            async with postgres_service.get_connection() as conn:
                result = await conn.fetchrow("SELECT * FROM users WHERE id = $1", user_id)
        """
        if not self.pool:
            await self.create_pool()

        async with self.pool.acquire() as connection:
            yield connection

    async def execute(self, query: str, *args) -> str:
        """
        Execute a query that doesn't return results (INSERT, UPDATE, DELETE).

        Args:
            query: SQL query
            *args: Query parameters

        Returns:
            Status message from database
        """
        try:
            async with self.get_connection() as conn:
                result = await conn.execute(query, *args)
                logger.debug(f"Executed query: {result}")
                return result
        except Exception as e:
            logger.error(f"Postgres execute error: {e}")
            raise

    async def fetch_one(self, query: str, *args) -> Optional[dict]:
        """
        Fetch a single row.

        Args:
            query: SQL query
            *args: Query parameters

        Returns:
            Row as dictionary or None
        """
        try:
            async with self.get_connection() as conn:
                row = await conn.fetchrow(query, *args)
                return dict(row) if row else None
        except Exception as e:
            logger.error(f"Postgres fetch_one error: {e}")
            raise

    async def fetch_many(self, query: str, *args) -> List[dict]:
        """
        Fetch multiple rows.

        Args:
            query: SQL query
            *args: Query parameters

        Returns:
            List of rows as dictionaries
        """
        try:
            async with self.get_connection() as conn:
                rows = await conn.fetch(query, *args)
                return [dict(row) for row in rows]
        except Exception as e:
            logger.error(f"Postgres fetch_many error: {e}")
            raise

    async def fetch_value(self, query: str, *args) -> Any:
        """
        Fetch a single value.

        Args:
            query: SQL query
            *args: Query parameters

        Returns:
            Single value
        """
        try:
            async with self.get_connection() as conn:
                value = await conn.fetchval(query, *args)
                return value
        except Exception as e:
            logger.error(f"Postgres fetch_value error: {e}")
            raise
