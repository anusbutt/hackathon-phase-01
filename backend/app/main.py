"""
FastAPI Application Entry Point for RAG Chatbot

This is the main application file that initializes FastAPI and configures
all middleware, routers, and startup/shutdown events.
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from datetime import datetime
import logging

from app.api.chat import router as chat_router, initialize_chat_services

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics RAG Chatbot API",
    description="Retrieval-Augmented Generation chatbot for the Physical AI textbook",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc"
)

# CORS Configuration
# TODO: Load from environment variables in Phase 1
CORS_ORIGINS = [
    "https://anusbutt.github.io",
    "http://localhost:3000",
    "http://localhost:3001"
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["Content-Type", "Authorization"]
)

# Include routers
app.include_router(chat_router)

# Health check endpoint
@app.get("/api/health")
async def health_check():
    """
    Health check endpoint for deployment verification.

    Returns:
        dict: Health status and timestamp
    """
    return {
        "status": "healthy",
        "timestamp": datetime.utcnow().isoformat(),
        "version": "1.0.0",
        "services": {
            "api": True
            # TODO: Add Qdrant, Gemini, Cohere health checks in Phase 1
        }
    }

@app.get("/")
async def root():
    """Root endpoint - redirects to docs."""
    return {
        "message": "Physical AI & Humanoid Robotics RAG Chatbot API",
        "docs": "/docs",
        "health": "/api/health"
    }

# Startup event
@app.on_event("startup")
async def startup_event():
    """Execute on application startup."""
    logger.info("[STARTUP] RAG Chatbot API starting up...")
    logger.info("[STARTUP] CORS enabled for: %s", CORS_ORIGINS)

    # Initialize chat services (RAG, Gemini, Cohere, Qdrant)
    try:
        initialize_chat_services()
        logger.info("[STARTUP] Chat services initialized successfully")
    except Exception as e:
        logger.error(f"[STARTUP] Failed to initialize chat services: {e}")
        raise

# Shutdown event
@app.on_event("shutdown")
async def shutdown_event():
    """Execute on application shutdown."""
    logger.info("[SHUTDOWN] RAG Chatbot API shutting down...")
    # TODO: Close database connections, cleanup resources in Phase 1
