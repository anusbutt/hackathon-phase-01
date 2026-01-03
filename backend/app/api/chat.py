"""
Chat API Endpoints

Handles chatbot query and feedback endpoints.
"""

from fastapi import APIRouter, HTTPException, status
from typing import Optional
import logging
import uuid
from datetime import datetime, timedelta, timezone

from app.models.schemas import ChatQueryRequest, ChatQueryResponse, ChatFeedbackRequest, Message
from app.services.rag_service import RAGService
from app.services.cohere_service import CohereService
from app.services.qdrant_service import QdrantService
from app.config.settings import get_settings

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/api/chat", tags=["chat"])

# Initialize services (will be done at startup)
rag_service: Optional[RAGService] = None

# Conversation expiry duration (7 days)
CONVERSATION_EXPIRY_DAYS = 7


def validate_conversation_expiry(conversation_created_at: Optional[datetime]) -> None:
    """
    Validate that a conversation has not expired.

    Conversations expire after 7 days from creation.

    Args:
        conversation_created_at: When the conversation was created

    Raises:
        HTTPException: If conversation has expired
    """
    if not conversation_created_at:
        return  # New conversation, no expiry check needed

    now = datetime.now(timezone.utc)
    expiry_date = conversation_created_at + timedelta(days=CONVERSATION_EXPIRY_DAYS)

    if now > expiry_date:
        logger.warning(f"Conversation expired (created: {conversation_created_at}, now: {now})")
        raise HTTPException(
            status_code=status.HTTP_410_GONE,
            detail=f"Conversation has expired. Conversations are only valid for {CONVERSATION_EXPIRY_DAYS} days."
        )


def initialize_chat_services():
    """Initialize chat services (called at app startup)."""
    global rag_service

    settings = get_settings()

    # Initialize component services
    cohere_service = CohereService(
        api_key=settings.cohere_api_key,
        model=settings.cohere_embedding_model
    )

    qdrant_service = QdrantService(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key,
        collection_name=settings.qdrant_collection_name
    )

    # Initialize RAG service
    rag_service = RAGService(
        cohere_service=cohere_service,
        qdrant_service=qdrant_service,
        gemini_api_key=settings.gemini_api_key,
        gemini_base_url=settings.gemini_base_url,
        gemini_model=settings.gemini_model
    )

    logger.info("Chat services initialized successfully")


@router.post("/query", response_model=ChatQueryResponse)
async def chat_query(request: ChatQueryRequest):
    """
    Process a chatbot query with RAG.

    **Request Body:**
    - `query`: User's question (1-2000 chars)
    - `selected_text`: Optional text selected by user
    - `lesson_id`: Optional current lesson ID for filtering
    - `conversation_history`: Last 5 messages for context

    **Response:**
    - `response`: AI-generated answer
    - `sources`: List of source citations
    - `conversation_id`: Unique conversation identifier
    - `timestamp`: Response timestamp

    **Error Responses:**
    - `400`: Invalid request (empty query, etc.)
    - `429`: Rate limit exceeded
    - `500`: Internal server error (LLM API failure, etc.)
    """
    try:
        # Validate service initialization
        if rag_service is None:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Chat service not initialized"
            )

        # Validate conversation expiry (if continuing existing conversation)
        validate_conversation_expiry(request.conversation_created_at)

        # Log request
        logger.info(f"Chat query received: {request.query[:100]}...")
        if request.conversation_id:
            logger.info(f"Continuing conversation: {request.conversation_id}")

        # Process query through RAG pipeline
        result = await rag_service.query(
            user_query=request.query,
            conversation_history=request.conversation_history,
            selected_text=request.selected_text,
            lesson_id=request.lesson_id,
            top_k=5,
            score_threshold=0.5  # Lower threshold for testing with mock embeddings
        )

        # Use existing conversation ID or generate new one
        conversation_id = request.conversation_id or str(uuid.uuid4())

        # Build response
        response = ChatQueryResponse(
            response=result['response'],
            sources=result['sources'],
            conversation_id=conversation_id,
            timestamp=datetime.now(timezone.utc),
            retrieved_chunks=result.get('retrieved_chunks'),
            used_selected_text=result.get('used_selected_text')
        )

        logger.info(f"Response generated ({len(result['sources'])} sources)")
        return response

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Chat query error: {e}", exc_info=True)

        # Handle specific Qdrant filter errors (filter not supported due to missing index)
        error_msg = str(e)
        if "Unexpected Response: 400" in error_msg and "Index required" in error_msg:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="The lesson filter is not available yet. Please search without lesson_id for now."
            )

        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to process query: {str(e)}"
        )


@router.post("/feedback")
async def chat_feedback(request: ChatFeedbackRequest):
    """
    Submit feedback for a conversation.

    **Request Body:**
    - `conversation_id`: ID of the conversation
    - `rating`: Rating from 1-5
    - `comment`: Optional feedback comment (max 500 chars)

    **Response:**
    - Success message

    **Error Responses:**
    - `400`: Invalid request (invalid rating, etc.)
    - `500`: Internal server error
    """
    try:
        logger.info(f"Feedback received: conversation={request.conversation_id}, rating={request.rating}")

        # TODO: Store feedback in database (Phase 2 enhancement)
        # For now, just log it
        if request.comment:
            logger.info(f"Feedback comment: {request.comment}")

        return {
            "message": "Feedback received successfully",
            "conversation_id": request.conversation_id
        }

    except Exception as e:
        logger.error(f"Feedback error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to submit feedback"
        )
