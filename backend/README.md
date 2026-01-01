# RAG Chatbot Backend

Backend API for the Physical AI & Humanoid Robotics book chatbot.

## Technology Stack

- **Framework**: FastAPI (Python 3.11+)
- **LLM**: Google Gemini (gemini-2.5-flash) via OpenAI Agents SDK
- **Embeddings**: Cohere (embed-english-v3.0)
- **Vector DB**: Qdrant Cloud Free Tier
- **Database**: Neon Serverless Postgres
- **Deployment**: Railway

## Quick Start (Phase 0 - Local Development)

### Prerequisites

- Python 3.11 or higher
- pip (Python package manager)

### Setup

1. **Create virtual environment**:
   ```bash
   cd backend
   python -m venv venv

   # On Windows
   venv\Scripts\activate

   # On macOS/Linux
   source venv/bin/activate
   ```

2. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Run the development server**:
   ```bash
   uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
   ```

4. **Verify it's working**:
   - Open browser: http://localhost:8000
   - Health check: http://localhost:8000/api/health
   - API docs: http://localhost:8000/docs

### Expected Response

Health check should return:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-30T...",
  "version": "1.0.0",
  "services": {
    "api": true
  }
}
```

## Phase 1 Setup (Coming Next)

After Phase 0 validation:
- Create `.env` file from `.env.example`
- Add API keys (Gemini, Cohere, Qdrant, Neon)
- Initialize external services

## Project Structure

```
backend/
├── app/
│   ├── __init__.py
│   ├── main.py                  # FastAPI entry point
│   ├── config/
│   │   └── settings.py          # Environment variables
│   ├── models/
│   │   ├── schemas.py           # Pydantic request/response models
│   │   └── entities.py          # Domain entities
│   ├── agents/
│   │   └── chatbot_agent.py     # OpenAI Agent with Gemini
│   ├── tools/
│   │   ├── search_content.py    # Qdrant search tool
│   │   ├── selected_text.py     # Selected text context tool
│   │   └── ...
│   ├── services/
│   │   ├── qdrant_service.py    # Qdrant client wrapper
│   │   ├── cohere_service.py    # Cohere embeddings
│   │   └── postgres_service.py  # Neon Postgres
│   ├── routers/
│   │   ├── chat.py              # Chat endpoints
│   │   └── health.py            # Health check
│   └── middleware/
│       ├── cors.py              # CORS configuration
│       ├── rate_limit.py        # Rate limiting
│       └── error_handler.py     # Error handling
├── scripts/
│   ├── extract_lessons.py       # Content extraction
│   ├── generate_embeddings.py   # Embedding generation
│   └── setup_qdrant.py          # Qdrant setup
├── tests/
│   ├── unit/
│   ├── integration/
│   └── e2e/
├── requirements.txt
├── requirements-dev.txt
├── .env.example
└── README.md                    # This file
```

## Development

### Code Quality

```bash
# Install dev dependencies
pip install -r requirements-dev.txt

# Format code
black app/ tests/

# Lint code
ruff app/ tests/

# Type check
mypy app/
```

### Testing

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=app tests/

# Run specific test file
pytest tests/unit/test_tools.py
```

## API Documentation

Once running, visit:
- **Swagger UI**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc

## Environment Variables

See `.env.example` for all required environment variables.

**Never commit `.env` file to git!**

## Deployment

See `specs/005-rag-chatbot/deployment.md` for deployment instructions.

## Status

- ✅ Phase 0: Project Setup - COMPLETE
- ⏳ Phase 1: Foundational Infrastructure - PENDING
- ⏳ Phase 2: Content Embedding - PENDING
- ⏳ Phase 3+: User Stories - PENDING
