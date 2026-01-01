"""
Domain Entities for RAG Chatbot

Core business entities used throughout the application.
"""

from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime, timedelta


class ChunkMetadata(BaseModel):
    """
    Metadata for a content chunk stored in Qdrant.
    """
    module: str = Field(..., description="Module ID (e.g., '01-ros2-nervous-system')")
    lesson: str = Field(..., description="Lesson ID (e.g., '02-nodes-topics-services')")
    section: str = Field(..., description="Section heading (e.g., '## What Is a ROS2 Node?')")
    tags: List[str] = Field(default_factory=list, description="Tags from frontmatter")
    skills: List[str] = Field(default_factory=list, description="Skills from frontmatter")
    bloom_level: Optional[str] = Field(None, description="Bloom's taxonomy level")
    lesson_title: str = Field(..., description="Human-readable lesson title")
    module_title: str = Field(..., description="Human-readable module title")


class ContentChunk(BaseModel):
    """
    A chunk of book content with embedding.

    Represents a semantic section of a lesson (500-1000 tokens).
    """
    chunk_id: str = Field(..., description="Unique chunk identifier (UUID)")
    content: str = Field(..., description="Chunk text content")
    embedding: List[float] = Field(..., description="1024-dimensional vector from Cohere")
    metadata: ChunkMetadata = Field(..., description="Chunk metadata")


class Conversation(BaseModel):
    """
    A multi-turn conversation between user and chatbot.

    Stored in browser localStorage for anonymous users.
    """
    conversation_id: str = Field(..., description="Unique conversation identifier (UUID)")
    messages: List = Field(default_factory=list, description="List of Message objects")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="When conversation started")
    expires_at: datetime = Field(..., description="When conversation expires (7 days)")

    @classmethod
    def create_new(cls, conversation_id: str):
        """Create a new conversation with 7-day expiry."""
        now = datetime.utcnow()
        return cls(
            conversation_id=conversation_id,
            messages=[],
            created_at=now,
            expires_at=now + timedelta(days=7)
        )

    def is_expired(self) -> bool:
        """Check if conversation has expired."""
        return datetime.utcnow() > self.expires_at

    def add_message(self, message):
        """Add a message to the conversation."""
        self.messages.append(message)

    def get_last_n_messages(self, n: int = 5):
        """Get the last N messages from conversation."""
        return self.messages[-n:] if len(self.messages) > n else self.messages


class SelectedTextContext(BaseModel):
    """
    Context for user-selected text on a lesson page.
    """
    selected_text: str = Field(..., description="The text user selected")
    lesson_id: str = Field(..., description="Lesson where text was selected")
    surrounding_context: str = Field(..., description="Paragraphs before/after selection")
    lesson_metadata: Optional[ChunkMetadata] = Field(None, description="Metadata from source lesson")
