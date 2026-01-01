"""
Cohere Service - Embeddings Generation

Handles all interactions with Cohere API for generating embeddings.
"""

import cohere
from typing import List
import logging

logger = logging.getLogger(__name__)


class CohereService:
    """
    Service class for Cohere embeddings generation.

    Provides methods for generating embeddings for queries and documents.
    """

    def __init__(self, api_key: str, model: str = "embed-english-v3.0"):
        """
        Initialize Cohere client.

        Args:
            api_key: Cohere API key
            model: Embedding model to use (default: embed-english-v3.0)
        """
        self.client = cohere.Client(api_key=api_key)
        self.model = model
        logger.info(f"Cohere service initialized with model: {model}")

    def embed_query(self, text: str) -> List[float]:
        """
        Generate embedding for a search query.

        Args:
            text: Query text to embed

        Returns:
            Embedding vector (1024 dimensions for embed-english-v3.0)
        """
        try:
            response = self.client.embed(
                texts=[text],
                model=self.model,
                input_type="search_query"  # Optimized for queries
            )

            embedding = response.embeddings[0]
            logger.info(f"Generated query embedding (dim: {len(embedding)})")
            return embedding

        except Exception as e:
            logger.error(f"Cohere query embedding error: {e}")
            raise

    def embed_documents(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple documents.

        Args:
            texts: List of document texts to embed

        Returns:
            List of embedding vectors
        """
        try:
            response = self.client.embed(
                texts=texts,
                model=self.model,
                input_type="search_document"  # Optimized for indexing
            )

            embeddings = response.embeddings
            logger.info(f"Generated {len(embeddings)} document embeddings")
            return embeddings

        except Exception as e:
            logger.error(f"Cohere document embedding error: {e}")
            raise

    def embed_documents_batch(
        self,
        texts: List[str],
        batch_size: int = 96
    ) -> List[List[float]]:
        """
        Generate embeddings for documents in batches (for large datasets).

        Args:
            texts: List of document texts to embed
            batch_size: Number of texts per batch (default: 96, Cohere's recommended size)

        Returns:
            List of embedding vectors
        """
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            batch_embeddings = self.embed_documents(batch)
            all_embeddings.extend(batch_embeddings)

            logger.info(f"Processed batch {i // batch_size + 1}/{(len(texts) + batch_size - 1) // batch_size}")

        return all_embeddings
