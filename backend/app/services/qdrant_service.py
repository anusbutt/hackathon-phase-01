"""
Qdrant Service - Vector Database Client Wrapper

Handles all interactions with Qdrant Cloud for semantic search.
"""

from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue, PointStruct
from typing import List, Dict, Optional
import logging

logger = logging.getLogger(__name__)


class QdrantService:
    """
    Service class for Qdrant vector database operations.

    Provides methods for searching vectors and inserting content chunks.
    """

    def __init__(self, url: str, api_key: str, collection_name: str):
        """
        Initialize Qdrant client.

        Args:
            url: Qdrant Cloud cluster URL
            api_key: Qdrant API key
            collection_name: Name of the collection to use
        """
        self.client = QdrantClient(url=url, api_key=api_key)
        self.collection_name = collection_name
        logger.info(f"Qdrant service initialized for collection: {collection_name}")

    def search(
        self,
        query_vector: List[float],
        limit: int = 10,
        score_threshold: float = 0.7,
        filters: Optional[Dict[str, str]] = None
    ) -> List[Dict]:
        """
        Search for similar vectors in Qdrant.

        Args:
            query_vector: The embedding vector to search for (1024 dims)
            limit: Maximum number of results to return
            score_threshold: Minimum similarity score (0-1)
            filters: Optional metadata filters (e.g., {"module": "01-ros2"})

        Returns:
            List of search results with content and metadata
        """
        try:
            # Build filter if provided
            query_filter = None
            if filters:
                conditions = [
                    FieldCondition(
                        key=key,
                        match=MatchValue(value=value)
                    )
                    for key, value in filters.items()
                ]
                query_filter = Filter(must=conditions)

            # Perform search (use query_points in newer Qdrant versions)
            results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=limit,
                score_threshold=score_threshold,
                query_filter=query_filter
            )

            # Format results
            formatted_results = []
            for result in results.points:
                formatted_results.append({
                    "content": result.payload.get("content", ""),
                    "score": result.score,
                    "module": result.payload.get("module_title", ""),
                    "lesson": result.payload.get("lesson_title", ""),
                    "section": result.payload.get("section", ""),
                    "metadata": result.payload
                })

            logger.info(f"Qdrant search returned {len(formatted_results)} results (threshold: {score_threshold})")
            return formatted_results

        except Exception as e:
            logger.error(f"Qdrant search error: {e}")
            raise

    def insert_chunks(self, chunks: List[Dict]) -> bool:
        """
        Insert content chunks into Qdrant.

        Args:
            chunks: List of chunks with id, vector, and payload

        Returns:
            True if successful
        """
        try:
            points = [
                PointStruct(
                    id=chunk["id"],
                    vector=chunk["vector"],
                    payload=chunk["payload"]
                )
                for chunk in chunks
            ]

            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            logger.info(f"Inserted {len(chunks)} chunks into Qdrant")
            return True

        except Exception as e:
            logger.error(f"Qdrant insert error: {e}")
            raise

    def count_points(self) -> int:
        """
        Get total number of points in collection.

        Returns:
            Number of points
        """
        try:
            info = self.client.get_collection(self.collection_name)
            return info.points_count
        except Exception as e:
            logger.error(f"Qdrant count error: {e}")
            raise
