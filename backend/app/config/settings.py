"""
Application Settings and Environment Configuration

Loads and validates environment variables using Pydantic Settings.
All API keys and sensitive configuration must be in .env file.
"""

from pydantic_settings import BaseSettings
from typing import List
import os


class Settings(BaseSettings):
    """
    Application settings loaded from environment variables.

    All fields are loaded from .env file or environment variables.
    Required fields will raise ValidationError if missing.
    """

    # Google Gemini Configuration (LLM Provider)
    gemini_api_key: str
    gemini_base_url: str = "https://generativelanguage.googleapis.com/v1beta/openai/"
    gemini_model: str = "gemini-2.5-flash"

    # Cohere Configuration (Embeddings Provider)
    cohere_api_key: str
    cohere_embedding_model: str = "embed-english-v3.0"

    # Qdrant Cloud Configuration
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "humanoid_robotics_book"

    # Neon Postgres Configuration
    neon_database_url: str

    # API Configuration
    cors_origins: str = "https://anusbutt.github.io,http://localhost:3000"
    api_rate_limit: int = 100

    # Computed properties
    @property
    def cors_origins_list(self) -> List[str]:
        """Parse CORS origins from comma-separated string."""
        return [origin.strip() for origin in self.cors_origins.split(",")]

    class Config:
        """Pydantic configuration."""
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = False


# Global settings instance
# This will fail on import if required environment variables are missing
try:
    settings = Settings()
except Exception as e:
    # During Phase 0, .env may not exist yet - use placeholder
    print(f"⚠️  Settings not loaded (expected in Phase 0): {e}")
    print("    Create .env file with required variables before Phase 1")
    settings = None  # type: ignore


def get_settings() -> Settings:
    """
    Get the global settings instance.

    Returns:
        Settings instance

    Raises:
        RuntimeError: If settings failed to load
    """
    if settings is None:
        raise RuntimeError("Settings not loaded - check .env file")
    return settings
