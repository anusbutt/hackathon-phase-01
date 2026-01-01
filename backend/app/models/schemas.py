"""
Pydantic Models for API Request/Response Schemas

All request and response models for the RAG chatbot API.
"""

from pydantic import BaseModel, Field, validator
from typing import List, Optional, Literal
from datetime import datetime


class Source(BaseModel):
    """
    Source citation for chatbot response.

    Indicates which module/lesson the information came from.
    """
    module: str = Field(..., description="Module name (e.g., 'Module 1: ROS2 Nervous System')")
    lesson: str = Field(..., description="Lesson name (e.g., 'Lesson 2: Nodes and Topics')")
    section: Optional[str] = Field(None, description="Section within lesson")


class Message(BaseModel):
    """
    A single message in a conversation (user or assistant).
    """
    role: Literal["user", "assistant"] = Field(..., description="Message sender")
    content: str = Field(..., description="Message content")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="When message was created")
    sources: Optional[List[Source]] = Field(None, description="Source citations (assistant only)")
    selected_text: Optional[str] = Field(None, description="User-selected text (user only)")


class ChatQueryRequest(BaseModel):
    """
    Request model for POST /api/chat/query
    """
    query: str = Field(..., min_length=1, max_length=2000, description="User's question")
    selected_text: Optional[str] = Field(None, description="Text selected by user on page")
    lesson_id: Optional[str] = Field(None, description="Current lesson ID (e.g., 'module-01/lesson-02')")
    conversation_history: List[Message] = Field(default_factory=list, description="Last 5 messages")
    conversation_id: Optional[str] = Field(None, description="Existing conversation ID (for multi-turn)")
    conversation_created_at: Optional[datetime] = Field(None, description="When conversation was created (for expiry check)")

    @validator("query")
    def validate_query(cls, v):
        """Validate query is not empty after stripping whitespace."""
        if not v.strip():
            raise ValueError("Query cannot be empty")
        return v.strip()

    @validator("conversation_history")
    def validate_history_length(cls, v):
        """Limit conversation history to last 5 messages."""
        if len(v) > 5:
            return v[-5:]  # Keep only last 5
        return v


class ChatQueryResponse(BaseModel):
    """
    Response model for POST /api/chat/query
    """
    response: str = Field(..., description="Chatbot's answer")
    sources: List[Source] = Field(default_factory=list, description="Source citations")
    conversation_id: str = Field(..., description="Unique conversation ID")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Response timestamp")


class ChatFeedbackRequest(BaseModel):
    """
    Request model for POST /api/chat/feedback
    """
    conversation_id: str = Field(..., description="ID of the conversation being rated")
    rating: int = Field(..., ge=1, le=5, description="Rating from 1-5")
    comment: Optional[str] = Field(None, max_length=500, description="Optional feedback comment")


class HealthResponse(BaseModel):
    """
    Response model for GET /api/health
    """
    status: Literal["healthy", "degraded", "unhealthy"] = Field(..., description="Overall health status")
    services: dict = Field(..., description="Status of individual services")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Health check timestamp")
    version: str = Field(..., description="API version")
