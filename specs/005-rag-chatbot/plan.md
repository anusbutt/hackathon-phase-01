# Implementation Plan: RAG Chatbot for Physical AI Book

**Branch**: `005-rag-chatbot` | **Date**: 2025-12-30 | **Spec**: [spec.md](./spec.md)

**Input**: Feature specification from `/specs/005-rag-chatbot/spec.md`

---

## Summary

Build and embed a Retrieval-Augmented Generation (RAG) chatbot within the published Physical AI & Humanoid Robotics book. The chatbot will:

1. **Answer general questions** about book content using semantic search across all 4 modules (16 lessons)
2. **Answer questions about user-selected text** with contextual awareness
3. **Maintain multi-turn conversations** with last 5 messages as context
4. **Store conversation history** in browser localStorage (7-day expiry) for anonymous users

**Technical Approach**:
- **LLM**: Google Gemini (gemini-2.5-flash) via OpenAI Agents SDK with custom tools
- **Embeddings**: Cohere (embed-english-v3.0) for semantic content vectors
- **Vector DB**: Qdrant Cloud Free Tier for RAG content retrieval
- **Backend**: FastAPI (Python 3.11+) deployed to Railway
- **Frontend**: React/TypeScript components embedded in Docusaurus site
- **Storage**: Browser localStorage (anonymous users), Neon Postgres (future auth)

---

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript 5.6+ (frontend)

**Primary Dependencies**:
- Backend: FastAPI 0.100+, OpenAI Agents SDK, `openai` (AsyncOpenAI client), `cohere`, `qdrant-client`, `asyncpg`, Pydantic v2
- Frontend: React 19.0.0, Docusaurus 3.9.2, TypeScript 5.6.2

**Storage**:
- Qdrant Cloud Free Tier (1GB) - vector embeddings for book content
- Neon Serverless Postgres (0.5GB) - user data (minimal usage in Phase 2)
- Browser localStorage - anonymous user conversation history (7-day expiry)

**Testing**:
- Backend: pytest (unit), pytest-asyncio (async tests), httpx (API tests)
- Frontend: Jest (unit), Playwright (E2E)
- Integration: End-to-end tests for chat flow

**Target Platform**:
- Backend: Railway (Python runtime, auto-deploy from GitHub)
- Frontend: GitHub Pages (static site with embedded chat)
- Databases: Qdrant Cloud, Neon Serverless Postgres

**Project Type**: Web application (backend API + frontend components)

**Performance Goals**:
- Response time: <3 seconds (95th percentile) end-to-end
- Qdrant search: <500ms (95th percentile)
- Gemini LLM: <2 seconds (95th percentile)
- Concurrent users: 50+ without degradation
- Chat interface load: <1 second to interactive

**Constraints**:
- Gemini API: 15 requests/min, 1500/day (free tier)
- Cohere API: 100 calls/min, 10,000/month (free tier)
- Qdrant Cloud: 1GB storage (free tier)
- Railway: $5/month usage credit (free tier)
- Browser localStorage: ~5-10MB per domain
- Rate limit: 100 requests/hour per user

**Scale/Scope**:
- Content: 16 lessons, ~320 embedded chunks (500-1000 tokens each)
- Users: Anonymous only (Phase 2), unlimited concurrent (within Railway limits)
- Conversations: Up to 7 days retention per user (browser-based)
- API endpoints: 3 primary (`/api/chat/query`, `/api/chat/feedback`, `/api/health`)

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

✅ **Phase 2 Core Principles Compliance**:

| Principle | Status | Evidence |
|-----------|--------|----------|
| I. Integrated RAG Architecture | ✅ PASS | Uses mandated tech stack: OpenAI Agents SDK, Gemini LLM, Cohere embeddings, FastAPI, Qdrant Cloud Free Tier, Neon Postgres, Railway |
| II. Tool-Based Data Retrieval | ✅ PASS | Plan includes 5 custom tools with `@function_tool` decorator: search_book_content, get_selected_text_context, get_lesson_metadata, get_user_profile, save_conversation |
| III. Pure RAG Implementation | ✅ PASS | Qdrant Cloud for vectors (no OpenAI vector storage), Cohere embeddings, chunking strategy defined (500-1000 tokens, H2/H3 sections) |
| IV. Frontend-Backend Integration | ✅ PASS | React components (ChatInterface.tsx, TextSelectionHandler.tsx), API endpoints defined, SSE streaming planned |
| V. User Experience Focus | ✅ PASS | Anonymous users, browser localStorage (7-day retention), multi-turn conversations (last 5 messages), error handling with suggestions |

✅ **Quality Standards Compliance**:

| Standard | Target | Plan Addresses |
|----------|--------|----------------|
| Performance | <3s response, <500ms search | Architecture includes caching, async operations, connection pooling |
| Security | API keys secured, CORS restricted, input sanitized | Environment variables for secrets, CORS policy, Pydantic validation |
| Testing | 90% unit test coverage, integration tests, E2E tests | Pytest for backend, Playwright for E2E, manual QA checklist |
| Content Accuracy | 95% verifiable responses, 0% hallucinations | RAG with similarity threshold >0.7, source citations mandatory |

✅ **Deployment Strategy Compliance**:
- Backend: Railway ✅
- Frontend: GitHub Pages (embedded in Docusaurus) ✅
- Environment variables: GEMINI_API_KEY, COHERE_API_KEY, QDRANT_API_KEY, NEON_DATABASE_URL ✅
- Auto-deploy: GitHub Actions for frontend, Railway GitHub integration for backend ✅

**No violations found. Constitution compliance: PASS**

---

## Project Structure

### Documentation (this feature)

```text
specs/005-rag-chatbot/
├── spec.md              # Feature specification (USER STORIES, ACCEPTANCE CRITERIA, REQUIREMENTS)
├── plan.md              # This file (ARCHITECTURE, DECISIONS, STRUCTURE)
├── tasks.md             # Task breakdown (created by /sp.tasks command)
├── contracts/           # Tool contracts and API schemas
│   ├── tool-search-book-content.md
│   ├── tool-get-selected-text-context.md
│   ├── tool-get-lesson-metadata.md
│   ├── tool-get-user-profile.md
│   ├── tool-save-conversation.md
│   └── api-endpoints.md
└── checklists/
    ├── deployment-checklist.md
    └── testing-checklist.md
```

### Source Code (repository root)

```text
hackathon-phase-01/
├── backend/                          # NEW: Phase 2 backend
│   ├── app/
│   │   ├── __init__.py
│   │   ├── main.py                   # FastAPI application entry point
│   │   ├── config/
│   │   │   ├── __init__.py
│   │   │   └── settings.py           # Environment variables, Pydantic BaseSettings
│   │   ├── models/
│   │   │   ├── __init__.py
│   │   │   ├── schemas.py            # Pydantic models for API requests/responses
│   │   │   └── entities.py           # Domain entities (Conversation, Message, Source)
│   │   ├── agents/
│   │   │   ├── __init__.py
│   │   │   ├── chatbot_agent.py      # OpenAI Agent initialization with Gemini
│   │   │   └── agent_config.py       # Agent system prompts and instructions
│   │   ├── tools/
│   │   │   ├── __init__.py
│   │   │   ├── search_content.py     # @function_tool for Qdrant search
│   │   │   ├── selected_text.py      # @function_tool for selected text context
│   │   │   ├── lesson_metadata.py    # @function_tool for frontmatter access
│   │   │   ├── user_profile.py       # @function_tool for Neon Postgres user data
│   │   │   └── conversation.py       # @function_tool for saving conversations
│   │   ├── services/
│   │   │   ├── __init__.py
│   │   │   ├── qdrant_service.py     # Qdrant client wrapper, search logic
│   │   │   ├── cohere_service.py     # Cohere embedding generation
│   │   │   └── postgres_service.py   # Neon Postgres connection pool
│   │   ├── routers/
│   │   │   ├── __init__.py
│   │   │   ├── chat.py               # POST /api/chat/query, POST /api/chat/feedback
│   │   │   └── health.py             # GET /api/health
│   │   └── middleware/
│   │       ├── __init__.py
│   │       ├── cors.py               # CORS configuration
│   │       ├── rate_limit.py         # Rate limiting (100 req/hour per session)
│   │       └── error_handler.py      # Global exception handling
│   ├── scripts/
│   │   ├── __init__.py
│   │   ├── extract_lessons.py        # Parse MDX files, extract content + frontmatter
│   │   ├── generate_embeddings.py    # Generate Cohere embeddings, upload to Qdrant
│   │   └── setup_qdrant.py           # Create Qdrant collection, configure schema
│   ├── tests/
│   │   ├── __init__.py
│   │   ├── conftest.py               # Pytest fixtures (mock clients, test data)
│   │   ├── unit/
│   │   │   ├── test_tools.py         # Unit tests for each @function_tool
│   │   │   ├── test_services.py      # Unit tests for Qdrant, Cohere, Postgres services
│   │   │   └── test_models.py        # Pydantic model validation tests
│   │   ├── integration/
│   │   │   ├── test_agent.py         # OpenAI Agent with mock tools
│   │   │   ├── test_qdrant.py        # Real Qdrant queries (test cluster)
│   │   │   └── test_api.py           # FastAPI endpoint tests (TestClient)
│   │   └── e2e/
│   │       └── test_chat_flow.py     # End-to-end chat flow (httpx async)
│   ├── requirements.txt              # Python dependencies
│   ├── requirements-dev.txt          # Dev dependencies (pytest, black, ruff)
│   ├── .env.example                  # Environment variable template
│   ├── Dockerfile                    # Optional: for local Docker testing
│   └── railway.toml                  # Railway deployment configuration
│
├── book-source/                      # EXISTING: Phase 1 Docusaurus site
│   ├── docs/                         # Educational content (Phase 1)
│   ├── src/
│   │   ├── components/
│   │   │   ├── ChatInterface.tsx     # NEW: Main chat UI component
│   │   │   ├── TextSelectionHandler.tsx  # NEW: Text selection tooltip
│   │   │   ├── ChatMessage.tsx       # NEW: Individual message component
│   │   │   └── ChatInput.tsx         # NEW: Input field with send button
│   │   ├── hooks/
│   │   │   ├── useChatState.ts       # NEW: Chat state management (localStorage)
│   │   │   ├── useConversationHistory.ts  # NEW: Conversation CRUD
│   │   │   └── useTextSelection.ts   # NEW: Text selection detection
│   │   ├── services/
│   │   │   └── chatApi.ts            # NEW: API client for backend
│   │   ├── types/
│   │   │   └── chat.ts               # NEW: TypeScript interfaces (Message, Conversation)
│   │   └── css/
│   │       └── chat.module.css       # NEW: Chat component styles
│   ├── static/
│   ├── docusaurus.config.ts          # UPDATED: Add backend URL env var
│   ├── package.json                  # UPDATED: Add new dependencies
│   └── tsconfig.json
│
├── specs/
│   └── 005-rag-chatbot/              # NEW: This feature's documentation
│       ├── spec.md
│       ├── plan.md                   # This file
│       ├── tasks.md                  # Created by /sp.tasks
│       └── contracts/
│
├── .github/
│   └── workflows/
│       ├── deploy.yml                # UPDATED: Add backend tests
│       └── backend-deploy.yml        # NEW: Railway deployment workflow
│
├── .gitignore                        # UPDATED: Add backend/.env, backend/__pycache__
└── README.md                         # UPDATED: Phase 2 documentation
```

**Structure Decision**: Web application (Option 2) with separate `backend/` and `book-source/` (frontend) directories. Backend is a new Python FastAPI service. Frontend extends existing Docusaurus site with React components. This separation aligns with constitution requirement for Railway (backend) and GitHub Pages (frontend) deployment.

---

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No violations found. This section is intentionally empty.**

---

## Architecture Overview

### High-Level System Design

```
┌─────────────────────────────────────────────────────────────────┐
│                    USER (Browser)                                │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  Docusaurus Site (GitHub Pages)                            │ │
│  │  - Lesson Pages (MDX)                                      │ │
│  │  - ChatInterface.tsx (bottom-right widget)                 │ │
│  │  - TextSelectionHandler.tsx (tooltip on selection)         │ │
│  │  - localStorage (conversation history, 7-day expiry)       │ │
│  └────────────────────────────────────────────────────────────┘ │
│                            ↓ HTTPS                               │
└────────────────────────────┼────────────────────────────────────┘
                             ↓
┌─────────────────────────────────────────────────────────────────┐
│            FastAPI Backend (Railway)                             │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  API Layer (routers/)                                      │ │
│  │  - POST /api/chat/query                                    │ │
│  │  - POST /api/chat/feedback                                 │ │
│  │  - GET /api/health                                         │ │
│  │  - Middleware: CORS, Rate Limiting, Error Handling         │ │
│  └────────────────────────────────────────────────────────────┘ │
│                            ↓                                     │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  OpenAI Agent (agents/)                                    │ │
│  │  - Gemini 2.5 Flash LLM (via AsyncOpenAI)                 │ │
│  │  - Custom Tools (@function_tool):                          │ │
│  │    1. search_book_content                                  │ │
│  │    2. get_selected_text_context                            │ │
│  │    3. get_lesson_metadata                                  │ │
│  │    4. get_user_profile                                     │ │
│  │    5. save_conversation                                    │ │
│  └────────────────────────────────────────────────────────────┘ │
│           ↓                  ↓                   ↓               │
│  ┌────────────────┐  ┌────────────────┐  ┌──────────────────┐  │
│  │ Qdrant Service │  │ Cohere Service │  │ Postgres Service │  │
│  │ (services/)    │  │ (services/)    │  │ (services/)      │  │
│  └────────────────┘  └────────────────┘  └──────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
           ↓                      ↓                      ↓
┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│  Qdrant Cloud    │  │  Cohere API      │  │  Neon Postgres   │
│  (Vector DB)     │  │  (Embeddings)    │  │  (User Data)     │
│  - 1GB Free Tier │  │  - Free Tier     │  │  - Free Tier     │
│  - 320 chunks    │  │  - embed-v3.0    │  │  - Minimal usage │
└──────────────────┘  └──────────────────┘  └──────────────────┘
```

### Data Flow: General Question

```
1. User types "What is ROS2?" in ChatInterface
   ↓
2. Frontend sends POST /api/chat/query
   Body: {
     query: "What is ROS2?",
     conversation_history: [...last 5 messages]
   }
   ↓
3. FastAPI receives request, validates input (Pydantic)
   ↓
4. Initialize OpenAI Agent with Gemini LLM + 5 custom tools
   ↓
5. Agent analyzes query, decides to call search_book_content tool
   ↓
6. search_book_content tool:
   a. Generates query embedding via Cohere API
   b. Searches Qdrant collection for top-10 chunks (similarity >0.7)
   c. Returns chunks with metadata (module, lesson, section)
   ↓
7. Agent receives retrieved chunks as tool output
   ↓
8. Agent generates response using Gemini LLM with:
   - User query
   - Retrieved chunks (context)
   - Conversation history (last 5 messages)
   - System prompt (educational tone, cite sources)
   ↓
9. Agent returns final response with sources
   ↓
10. FastAPI formats response:
    {
      response: "ROS2 is...",
      sources: [
        {module: "Module 1: ROS2 Nervous System", lesson: "Lesson 1"}
      ],
      conversation_id: "uuid",
      timestamp: "ISO 8601"
    }
   ↓
11. Frontend receives response, displays in chat UI
   ↓
12. Frontend saves message pair to localStorage (conversation history)
```

### Data Flow: Selected Text Question

```
1. User selects text on lesson page: "DDS is the middleware..."
   ↓
2. TextSelectionHandler detects selection ≥10 chars
   ↓
3. Tooltip appears with "Ask about this" button
   ↓
4. User clicks, types "What is DDS?"
   ↓
5. Frontend sends POST /api/chat/query
   Body: {
     query: "What is DDS?",
     selected_text: "DDS is the middleware...",
     lesson_id: "module-01/lesson-02",
     conversation_history: [...]
   }
   ↓
6. Agent receives request, calls get_selected_text_context tool
   ↓
7. get_selected_text_context tool:
   a. Receives selected_text + lesson_id
   b. Queries Qdrant for chunks from same lesson (metadata filter)
   c. Retrieves surrounding paragraphs (before/after selection)
   d. Returns enriched context with lesson metadata
   ↓
8. Agent ALSO calls search_book_content for general knowledge
   ↓
9. Agent generates response PRIORITIZING selected_text context
   ↓
10. Response includes source citation for selected text lesson
   ↓
11. Frontend displays response, highlights selected text connection
```

---

## Key Architectural Decisions

### Decision 1: OpenAI Agents SDK with Gemini LLM

**Choice**: Use OpenAI Agents SDK for orchestration, but Google Gemini (gemini-2.5-flash) as the LLM provider via AsyncOpenAI client.

**Rationale**:
- OpenAI Agents SDK provides robust tool orchestration, automatic tool calling, and conversation management
- Gemini 2.5 Flash offers cost-effective, fast responses (free tier: 15 req/min)
- AsyncOpenAI client provides OpenAI-compatible interface for Gemini API
- Decouples agent framework (OpenAI SDK) from LLM provider (Gemini), allowing future swaps

**Alternatives Considered**:
- Native Google AI SDK: Less mature tool-calling support, would require custom orchestration
- LangChain: Over-engineered for our use case, adds unnecessary complexity
- Custom agent loop: Reinventing the wheel, error-prone

**Implementation Pattern**:
```python
from agents import Agent, OpenAIChatCompletionsModel, AsyncOpenAI

client = AsyncOpenAI(
    api_key=os.getenv("GEMINI_API_KEY"),
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

llm_model = OpenAIChatCompletionsModel(
    model="gemini-2.5-flash",
    openai_client=client
)

agent = Agent(
    name="RAG_Assistant",
    model=llm_model,
    tools=[search_book_content, get_selected_text_context, ...]
)
```

**Impact**: Backend (`agents/chatbot_agent.py`), Constitution (LLM provider specification)

---

### Decision 2: Cohere for Embeddings

**Choice**: Use Cohere `embed-english-v3.0` model for all content embeddings and query embeddings.

**Rationale**:
- Cohere free tier: 100 calls/min, 10,000/month (sufficient for Phase 2)
- High-quality embeddings (1024 dimensions)
- Cost: $0 for free tier (vs OpenAI $0.00002 per 1k tokens)
- One-time embedding generation (320 chunks), then only query embeddings

**Alternatives Considered**:
- OpenAI text-embedding-3-small: Requires OpenAI API key, costs money
- Sentence Transformers (local): Slower, requires GPU for good performance
- Google Embeddings API: Less mature, fewer resources/documentation

**Implementation Pattern**:
```python
import cohere

co = cohere.Client(api_key=os.getenv("COHERE_API_KEY"))

response = co.embed(
    texts=["content chunk 1", "content chunk 2"],
    model="embed-english-v3.0",
    input_type="search_document"  # for indexing
)

embeddings = response.embeddings  # [[float * 1024], ...]
```

**Impact**: Backend (`services/cohere_service.py`, `scripts/generate_embeddings.py`), Qdrant schema (1024-dim vectors)

---

### Decision 3: Qdrant Cloud for Vector Storage

**Choice**: Use Qdrant Cloud Free Tier for all vector embeddings.

**Rationale**:
- Free tier: 1GB storage (sufficient for ~1M embeddings, we need ~320)
- Managed service (no infrastructure to maintain)
- Fast semantic search (<100ms for most queries)
- Native metadata filtering (module, lesson, tags)
- Python client with async support

**Alternatives Considered**:
- Pinecone: Free tier only 1 index, slower search
- Weaviate: More complex setup, overkill for our scale
- ChromaDB (local): Requires hosting, not suitable for Railway deployment
- PostgreSQL pgvector: Slower than dedicated vector DB, complex setup

**Implementation Pattern**:
```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Create collection (one-time setup)
client.create_collection(
    collection_name="humanoid_robotics_book",
    vectors_config=VectorParams(size=1024, distance=Distance.COSINE)
)

# Search
results = client.search(
    collection_name="humanoid_robotics_book",
    query_vector=query_embedding,  # [float * 1024]
    limit=10,
    score_threshold=0.7
)
```

**Impact**: Backend (`services/qdrant_service.py`, `scripts/setup_qdrant.py`), Constitution (Qdrant Cloud specification)

---

### Decision 4: Browser localStorage for Anonymous Users

**Choice**: Store conversation history in browser localStorage with 7-day expiry (no server-side persistence for anonymous users in Phase 2).

**Rationale**:
- Aligns with constitution requirement: "anonymous-only" for Phase 2
- No server-side session management needed (reduces backend complexity)
- Privacy-friendly (data stays on user's device)
- 7-day expiry prevents localStorage bloat
- Fast (no network latency for history retrieval)

**Alternatives Considered**:
- Server-side sessions: Requires authentication, violates "anonymous-only" requirement
- IndexedDB: More complex API, overkill for simple key-value storage
- sessionStorage: Lost on tab close, poor UX

**Implementation Pattern**:
```typescript
// Frontend: hooks/useConversationHistory.ts
const STORAGE_KEY = 'rag_chatbot_history';
const EXPIRY_DAYS = 7;

interface StoredConversation {
  messages: Message[];
  created_at: string;
  expires_at: string;
}

function saveConversation(messages: Message[]) {
  const conversation: StoredConversation = {
    messages,
    created_at: new Date().toISOString(),
    expires_at: new Date(Date.now() + EXPIRY_DAYS * 24 * 60 * 60 * 1000).toISOString()
  };

  localStorage.setItem(STORAGE_KEY, JSON.stringify(conversation));
}

function loadConversation(): Message[] {
  const stored = localStorage.getItem(STORAGE_KEY);
  if (!stored) return [];

  const conversation: StoredConversation = JSON.parse(stored);

  // Check expiry
  if (new Date(conversation.expires_at) < new Date()) {
    localStorage.removeItem(STORAGE_KEY);
    return [];
  }

  return conversation.messages;
}
```

**Impact**: Frontend (`hooks/useConversationHistory.ts`), Backend (no conversation storage endpoint needed for Phase 2)

---

### Decision 5: Multi-turn Context: Last 5 Messages

**Choice**: Include only the last 5 user-assistant message pairs in the agent's context when generating responses.

**Rationale**:
- Balances context retention with token limits
- Gemini 2.5 Flash: ~1M token context, but smaller context = faster + cheaper
- 5 messages ≈ 2.5 turns (user + assistant) ≈ ~1000 tokens (estimate)
- Prevents "context dilution" (older messages less relevant)
- Matches common chatbot UX patterns

**Alternatives Considered**:
- All messages: Expensive, slow, context dilution
- Last 3 messages: Too little context for complex follow-ups
- Last 10 messages: Excessive for educational Q&A

**Implementation Pattern**:
```python
# Backend: agents/chatbot_agent.py
def prepare_conversation_context(conversation_history: list[Message]) -> str:
    """Extract last 5 messages for agent context."""
    last_5 = conversation_history[-5:] if len(conversation_history) > 5 else conversation_history

    context = ""
    for msg in last_5:
        context += f"{msg.role.upper()}: {msg.content}\n"

    return context
```

**Impact**: Frontend (sends last 5 messages in API request), Backend (agent initialization)

---

### Decision 6: Text Selection: Minimum 10 Characters

**Choice**: Only trigger "Ask about this" tooltip when user selects ≥10 characters of text.

**Rationale**:
- Prevents false positives (accidental selections, single words)
- 10 chars ≈ 2-3 words (enough for meaningful context)
- Reduces noise (tooltip not appearing constantly)
- Aligns with common text selection UX patterns (e.g., Medium, Notion)

**Alternatives Considered**:
- 5 characters: Too sensitive, triggers on single words
- 20 characters: Misses short but meaningful selections (e.g., "ROS2 DDS")
- Word count (≥3 words): More complex to implement, language-dependent

**Implementation Pattern**:
```typescript
// Frontend: components/TextSelectionHandler.tsx
const MIN_SELECTION_LENGTH = 10;

function handleMouseUp() {
  const selection = window.getSelection();
  const selectedText = selection?.toString().trim() || '';

  if (selectedText.length >= MIN_SELECTION_LENGTH) {
    showTooltip(selectedText, selection.getRangeAt(0).getBoundingClientRect());
  } else {
    hideTooltip();
  }
}

document.addEventListener('mouseup', handleMouseUp);
```

**Impact**: Frontend (`components/TextSelectionHandler.tsx`)

---

### Decision 7: Similarity Threshold 0.7 for Qdrant Search

**Choice**: Filter Qdrant search results to only include chunks with cosine similarity ≥0.7.

**Rationale**:
- 0.7 is a standard threshold for "relevant" content in RAG systems
- Lower threshold (0.5): Includes irrelevant chunks, increases hallucination risk
- Higher threshold (0.8): Too strict, may miss relevant content with different phrasing
- Can be adjusted during testing if 0.7 proves too strict/lenient

**Alternatives Considered**:
- No threshold: Returns low-quality matches, confuses LLM
- Dynamic threshold: Complex to implement, premature optimization
- Top-k only (no threshold): May include irrelevant chunks

**Implementation Pattern**:
```python
# Backend: tools/search_content.py
SIMILARITY_THRESHOLD = 0.7

results = qdrant_client.search(
    collection_name="humanoid_robotics_book",
    query_vector=query_embedding,
    limit=10,
    score_threshold=SIMILARITY_THRESHOLD  # Filter results <0.7
)

if len(results) == 0:
    # No relevant content found, trigger error handling
    return suggest_alternative_topics()
```

**Impact**: Backend (`tools/search_content.py`, `services/qdrant_service.py`)

---

### Decision 8: Rate Limiting: 100 Requests/Hour per Session

**Choice**: Enforce rate limit of 100 requests per hour per user, tracked by session ID (anonymous users).

**Rationale**:
- Prevents abuse (spamming, DoS attacks)
- Protects free tier API limits (Gemini: 1500/day, Cohere: 10,000/month)
- 100/hour ≈ 1.66/min (reasonable for educational Q&A)
- Session-based (not IP-based) to support shared networks (e.g., university wifi)

**Alternatives Considered**:
- IP-based rate limiting: Blocks legitimate users on shared IPs
- Per-minute limiting: More complex, overkill for our use case
- No rate limiting: Vulnerable to abuse, API cost explosion

**Implementation Pattern**:
```python
# Backend: middleware/rate_limit.py
from fastapi import Request, HTTPException
from datetime import datetime, timedelta
import hashlib

RATE_LIMIT = 100  # requests per hour
WINDOW = timedelta(hours=1)
request_counts = {}  # {session_id: [(timestamp, count), ...]}

async def rate_limit_middleware(request: Request, call_next):
    # Generate session ID from User-Agent + IP (weak but sufficient for anonymous users)
    session_id = hashlib.md5(
        (request.headers.get("User-Agent", "") + request.client.host).encode()
    ).hexdigest()

    now = datetime.now()

    # Clean old entries
    if session_id in request_counts:
        request_counts[session_id] = [
            (ts, count) for ts, count in request_counts[session_id]
            if now - ts < WINDOW
        ]

    # Count requests in current window
    current_count = sum(count for ts, count in request_counts.get(session_id, []))

    if current_count >= RATE_LIMIT:
        raise HTTPException(
            status_code=429,
            detail="Too many requests. Please wait a moment and try again.",
            headers={"Retry-After": "3600"}
        )

    # Record this request
    if session_id not in request_counts:
        request_counts[session_id] = []
    request_counts[session_id].append((now, 1))

    response = await call_next(request)
    return response
```

**Impact**: Backend (`middleware/rate_limit.py`, `main.py`)

---

## Data Model

### Backend Entities

**Conversation** (browser localStorage):
```python
from pydantic import BaseModel
from datetime import datetime

class Conversation(BaseModel):
    conversation_id: str  # UUID
    messages: list[Message]
    created_at: datetime
    expires_at: datetime  # created_at + 7 days
```

**Message** (in Conversation):
```python
class Message(BaseModel):
    role: Literal["user", "assistant"]
    content: str
    timestamp: datetime
    sources: list[Source] | None = None  # Only for assistant messages
    selected_text: str | None = None     # Only for user messages with selected text
```

**Source** (in Message):
```python
class Source(BaseModel):
    module: str  # e.g., "Module 1: ROS2 Nervous System"
    lesson: str  # e.g., "Lesson 2: Nodes and Topics"
    section: str | None = None  # e.g., "Key Principles"
```

**ContentChunk** (Qdrant):
```python
class ContentChunk(BaseModel):
    chunk_id: str  # UUID
    content: str   # 500-1000 tokens of lesson content
    embedding: list[float]  # [float * 1024] from Cohere
    metadata: ChunkMetadata
```

**ChunkMetadata** (in ContentChunk):
```python
class ChunkMetadata(BaseModel):
    module: str           # e.g., "01-ros2-nervous-system"
    lesson: str           # e.g., "02-nodes-topics-services"
    section: str          # e.g., "## What Is a ROS2 Node?"
    tags: list[str]       # From frontmatter
    skills: list[str]     # From frontmatter
    bloom_level: str      # From frontmatter
    lesson_title: str     # Human-readable lesson name
    module_title: str     # Human-readable module name
```

### API Schemas

**POST /api/chat/query Request**:
```python
class ChatQueryRequest(BaseModel):
    query: str  # Max 2000 characters
    selected_text: str | None = None
    lesson_id: str | None = None  # e.g., "module-01/lesson-02"
    conversation_history: list[Message] = []  # Last 5 messages

    @validator("query")
    def validate_query_length(cls, v):
        if len(v) > 2000:
            raise ValueError("Query must be ≤2000 characters")
        if len(v.strip()) == 0:
            raise ValueError("Query cannot be empty")
        return v
```

**POST /api/chat/query Response**:
```python
class ChatQueryResponse(BaseModel):
    response: str
    sources: list[Source]
    conversation_id: str
    timestamp: datetime
```

**POST /api/chat/feedback Request**:
```python
class ChatFeedbackRequest(BaseModel):
    conversation_id: str
    rating: int  # 1-5
    comment: str | None = None
```

**GET /api/health Response**:
```python
class HealthResponse(BaseModel):
    status: Literal["healthy", "degraded", "unhealthy"]
    services: dict[str, bool]  # {"qdrant": True, "gemini": True, "cohere": True}
    timestamp: datetime
```

---

## Custom Tools (OpenAI Agents SDK)

### Tool 1: search_book_content

**Purpose**: Semantic search across all book content in Qdrant.

**Contract**:
```python
from agents import function_tool

@function_tool
def search_book_content(
    query: str,
    module_filter: str | None = None,
    top_k: int = 10
) -> list[dict]:
    """
    Search the book content using semantic similarity in Qdrant.

    Args:
        query: The user's question or search query
        module_filter: Optional module name to restrict search scope (e.g., "01-ros2-nervous-system")
        top_k: Number of results to return (default 10)

    Returns:
        List of relevant content chunks with metadata and similarity scores.
        Each item: {
            "content": str,
            "score": float,
            "module": str,
            "lesson": str,
            "section": str
        }
    """
    # 1. Generate query embedding via Cohere
    embedding = cohere_service.embed_query(query)

    # 2. Search Qdrant with filters
    filters = {"module": module_filter} if module_filter else None
    results = qdrant_service.search(
        collection_name="humanoid_robotics_book",
        query_vector=embedding,
        limit=top_k,
        score_threshold=0.7,
        filters=filters
    )

    # 3. Format results
    return [
        {
            "content": result.payload["content"],
            "score": result.score,
            "module": result.payload["module_title"],
            "lesson": result.payload["lesson_title"],
            "section": result.payload["section"]
        }
        for result in results
    ]
```

**Implementation File**: `backend/app/tools/search_content.py`

---

### Tool 2: get_selected_text_context

**Purpose**: Enrich user-selected text with surrounding lesson context.

**Contract**:
```python
@function_tool
def get_selected_text_context(
    selected_text: str,
    lesson_id: str
) -> dict:
    """
    Process user-selected text and retrieve surrounding context from the same lesson.

    Args:
        selected_text: The text the user selected on the page
        lesson_id: The lesson identifier (e.g., "module-01/lesson-02")

    Returns:
        {
            "selected_text": str,
            "surrounding_context": str,  # Paragraphs before/after
            "lesson_metadata": {
                "module": str,
                "lesson": str,
                "section": str,
                "tags": list[str]
            }
        }
    """
    # 1. Parse lesson_id to extract module/lesson
    module, lesson = lesson_id.split("/")

    # 2. Query Qdrant for chunks from same lesson
    results = qdrant_service.search_with_filter(
        collection_name="humanoid_robotics_book",
        filters={"module": module, "lesson": lesson},
        limit=5
    )

    # 3. Find chunk containing selected_text
    matching_chunk = next(
        (r for r in results if selected_text in r.payload["content"]),
        None
    )

    if not matching_chunk:
        # Fallback: return selected text with lesson metadata
        return {
            "selected_text": selected_text,
            "surrounding_context": "",
            "lesson_metadata": extract_metadata_from_lesson_id(lesson_id)
        }

    # 4. Extract surrounding paragraphs (before/after selected text)
    full_content = matching_chunk.payload["content"]
    paragraphs = full_content.split("\n\n")

    # Find paragraph containing selected_text
    selected_para_idx = next(
        (i for i, p in enumerate(paragraphs) if selected_text in p),
        0
    )

    # Get surrounding paragraphs (1 before, 1 after)
    start_idx = max(0, selected_para_idx - 1)
    end_idx = min(len(paragraphs), selected_para_idx + 2)
    surrounding = "\n\n".join(paragraphs[start_idx:end_idx])

    return {
        "selected_text": selected_text,
        "surrounding_context": surrounding,
        "lesson_metadata": matching_chunk.payload["metadata"]
    }
```

**Implementation File**: `backend/app/tools/selected_text.py`

---

### Tool 3: get_lesson_metadata

**Purpose**: Retrieve frontmatter metadata for a specific lesson (for enhanced context).

**Contract**:
```python
@function_tool
def get_lesson_metadata(lesson_id: str) -> dict:
    """
    Fetch lesson frontmatter metadata (learning objectives, skills, tags).

    Args:
        lesson_id: The lesson identifier (e.g., "module-01/lesson-02")

    Returns:
        {
            "title": str,
            "module": str,
            "lesson": str,
            "learning_objectives": list[dict],
            "skills": list[dict],
            "tags": list[str],
            "cognitive_load": dict
        }
    """
    # This can be cached in-memory or Redis since metadata is static
    # For Phase 2, use a simple JSON file mapping lesson_id -> metadata

    metadata_file = Path("backend/data/lesson_metadata.json")
    metadata_db = json.loads(metadata_file.read_text())

    return metadata_db.get(lesson_id, {})
```

**Implementation File**: `backend/app/tools/lesson_metadata.py`

**Data Source**: `backend/data/lesson_metadata.json` (generated during embedding pipeline)

---

### Tool 4: get_user_profile

**Purpose**: Retrieve user preferences and learning progress from Neon Postgres (future-proofing for Phase 3).

**Contract**:
```python
@function_tool
def get_user_profile(user_id: str) -> dict | None:
    """
    Retrieve user profile from Neon Postgres (if authenticated).

    Args:
        user_id: The user's unique identifier (UUID)

    Returns:
        {
            "username": str,
            "current_module": str,
            "current_lesson": str,
            "preferences": dict
        }

        Returns None for anonymous users (Phase 2).
    """
    # Phase 2: Always return None (no authentication)
    return None

    # Phase 3+: Query Neon Postgres
    # async with postgres_service.get_connection() as conn:
    #     user = await conn.fetchrow(
    #         "SELECT username, current_module, current_lesson, preferences FROM users WHERE id = $1",
    #         user_id
    #     )
    #     return dict(user) if user else None
```

**Implementation File**: `backend/app/tools/user_profile.py`

**Note**: Minimal implementation in Phase 2 (always returns None). Full implementation in future phases.

---

### Tool 5: save_conversation

**Purpose**: Store conversation history in Neon Postgres (future-proofing for Phase 3).

**Contract**:
```python
@function_tool
def save_conversation(
    user_id: str | None,
    query: str,
    response: str,
    selected_text: str | None,
    lesson_id: str | None,
    sources: list[dict]
) -> str:
    """
    Save conversation to Neon Postgres for analytics and future retrieval.

    Args:
        user_id: User identifier (None for anonymous users)
        query: User's question
        response: Chatbot's response
        selected_text: User-selected text (if applicable)
        lesson_id: Current lesson (if applicable)
        sources: List of source citations

    Returns:
        conversation_id (UUID as string)
    """
    # Phase 2: No-op (conversations only in browser localStorage)
    return str(uuid.uuid4())

    # Phase 3+: Insert into Neon Postgres
    # async with postgres_service.get_connection() as conn:
    #     conversation_id = await conn.fetchval(
    #         """
    #         INSERT INTO conversations (user_id, query, response, selected_text, lesson_id, sources)
    #         VALUES ($1, $2, $3, $4, $5, $6)
    #         RETURNING id
    #         """,
    #         user_id, query, response, selected_text, lesson_id, json.dumps(sources)
    #     )
    #     return str(conversation_id)
```

**Implementation File**: `backend/app/tools/conversation.py`

**Note**: Minimal implementation in Phase 2 (no database writes). Full implementation in future phases.

---

## Non-Functional Requirements

### Performance

**Response Time**:
- End-to-end: <3s (P95) from user submit to response displayed
- Breakdown:
  - Frontend → Backend: <100ms (network)
  - Query embedding (Cohere): <200ms
  - Qdrant search: <500ms (P95)
  - Agent processing (Gemini LLM): <2s (P95)
  - Backend → Frontend: <100ms (network)

**Concurrency**:
- Target: 50+ concurrent users without degradation
- Strategy:
  - Async FastAPI (handles concurrent requests efficiently)
  - Connection pooling for Qdrant and Neon Postgres
  - Rate limiting (100 req/hour per user)

**Caching**:
- Lesson metadata: In-memory cache (static data)
- Qdrant client: Persistent connection (no reconnection overhead)
- Cohere query embeddings: No caching (query-dependent)

---

### Security

**Input Validation**:
- Max query length: 2000 characters
- Pydantic models for all API requests (automatic validation)
- Sanitize inputs to prevent XSS, SQL injection
- HTML escaping in frontend (React default behavior)

**Authentication & Authorization**:
- Phase 2: Anonymous-only (no authentication)
- Session-based rate limiting (weak but sufficient)
- Phase 3+: Better-auth integration

**Secrets Management**:
- All API keys in Railway environment variables
- Never commit secrets to git (.env in .gitignore)
- .env.example for documentation only

**CORS Policy**:
- Allowed origins: `https://anusbutt.github.io`, `http://localhost:3000`
- Methods: POST, GET, OPTIONS
- Headers: Content-Type, Authorization (future)

**Error Handling**:
- No stack traces in production responses
- Generic error messages to users
- Detailed logging to Railway logs (server-side only)

---

### Reliability

**Error Recovery**:
- Retry logic for transient API failures (Gemini, Cohere, Qdrant)
- Graceful degradation when Qdrant is down (show error message, suggest offline FAQ)
- Health check endpoint for deployment verification

**Monitoring**:
- Railway logs for all requests, errors, warnings
- Optional: Metrics endpoint for response times, error rates (future)

**Data Integrity**:
- Conversation history: Browser localStorage (user-managed)
- Embeddings: One-time generation, immutable in Qdrant
- No database writes in Phase 2 (no data loss risk)

---

### Scalability

**Current Scope**:
- Users: Unlimited (within free tier API limits)
- Content: 16 lessons, ~320 chunks (static)
- Conversations: Browser-only (no server storage)

**Scaling Strategy** (future phases):
- Railway: Upgrade to paid tier ($5-20/month for higher limits)
- Qdrant: Migrate to paid tier if >1GB needed
- Caching: Add Redis for frequent queries
- Database: Upgrade Neon Postgres to paid tier

**API Rate Limit Management**:
- Gemini: 15 req/min → Use request queue, show "Please wait" to users
- Cohere: 100 req/min → Only used for query embeddings (should never hit limit)
- Qdrant: No limit (managed service scales automatically)

---

## Testing Strategy

### Unit Tests (pytest)

**Backend**:
- `tests/unit/test_tools.py`: Test each `@function_tool` in isolation with mock clients
- `tests/unit/test_services.py`: Test Qdrant, Cohere, Postgres service wrappers
- `tests/unit/test_models.py`: Pydantic model validation (edge cases, invalid inputs)

**Frontend**:
- `tests/unit/hooks.test.ts`: Test conversation history CRUD (localStorage)
- `tests/unit/chatApi.test.ts`: Test API client with mock fetch

**Coverage Target**: 90% for backend, 80% for frontend

---

### Integration Tests

**Backend** (`tests/integration/`):
- `test_agent.py`: Test OpenAI Agent with mock tools (verify tool calling logic)
- `test_qdrant.py`: Test real Qdrant queries (use test cluster or mock)
- `test_api.py`: Test FastAPI endpoints with TestClient (httpx)

**Example**:
```python
from fastapi.testclient import TestClient
from app.main import app

client = TestClient(app)

def test_chat_query_success():
    response = client.post("/api/chat/query", json={
        "query": "What is ROS2?",
        "conversation_history": []
    })

    assert response.status_code == 200
    data = response.json()
    assert "response" in data
    assert "sources" in data
    assert len(data["sources"]) > 0
```

---

### End-to-End Tests (Playwright)

**Frontend** (`tests/e2e/`):
- Test full chat flow: open interface → type query → receive response
- Test selected text flow: select text → click tooltip → ask question
- Test multi-turn conversation: ask 3 follow-up questions
- Test error handling: ask out-of-scope question → see helpful error

**Example**:
```typescript
import { test, expect } from '@playwright/test';

test('user can ask a question and receive a response', async ({ page }) => {
  await page.goto('http://localhost:3000/docs/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/01-ros2-fundamentals');

  // Open chat interface
  await page.click('[data-testid="chat-toggle"]');

  // Type question
  await page.fill('[data-testid="chat-input"]', 'What is ROS2?');
  await page.click('[data-testid="chat-send"]');

  // Wait for response
  await page.waitForSelector('[data-testid="chat-message-assistant"]', { timeout: 5000 });

  // Verify response contains expected content
  const response = await page.textContent('[data-testid="chat-message-assistant"]');
  expect(response).toContain('ROS2');

  // Verify source citation
  const source = await page.textContent('[data-testid="chat-source"]');
  expect(source).toContain('Module 1');
});
```

---

### Manual QA Checklist

Before deployment:
- [ ] Test 10 sample questions across all 4 modules
- [ ] Test selected text feature on 5 different lessons
- [ ] Test multi-turn conversation (5+ messages)
- [ ] Test error handling (out-of-scope query, API failure simulation)
- [ ] Test rate limiting (send 101 requests in 1 hour)
- [ ] Test browser compatibility (Chrome, Firefox, Safari, Edge)
- [ ] Test mobile responsiveness (text selection, chat interface)
- [ ] Test dark mode compatibility
- [ ] Verify CORS policy (frontend on GitHub Pages → backend on Railway)
- [ ] Verify no secrets in git history (`git log | grep -i "api_key"`)

---

## Deployment Plan

### Phase 0: Local Development Setup

1. **Clone repository**:
   ```bash
   git clone https://github.com/anusbutt/hackathon-phase-01.git
   cd hackathon-phase-01
   ```

2. **Backend setup**:
   ```bash
   cd backend
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r requirements.txt
   pip install -r requirements-dev.txt

   # Copy .env.example to .env and fill in API keys
   cp .env.example .env
   # Edit .env with your keys
   ```

3. **Frontend setup**:
   ```bash
   cd ../book-source
   npm install
   ```

4. **Run locally**:
   ```bash
   # Terminal 1 (backend)
   cd backend
   uvicorn app.main:app --reload --host 0.0.0.0 --port 8000

   # Terminal 2 (frontend)
   cd book-source
   npm start
   ```

5. **Verify**:
   - Backend: http://localhost:8000/api/health
   - Frontend: http://localhost:3000

---

### Phase 1: Qdrant Cloud Setup

1. **Create Qdrant Cloud account** (free tier):
   - Visit: https://cloud.qdrant.io/
   - Sign up (no credit card required)

2. **Create collection**:
   ```bash
   cd backend
   python scripts/setup_qdrant.py
   ```

3. **Verify**:
   ```bash
   python scripts/test_qdrant.py
   ```

---

### Phase 2: Content Embedding Pipeline

1. **Extract lessons**:
   ```bash
   cd backend
   python scripts/extract_lessons.py
   # Output: backend/data/extracted_lessons.json
   ```

2. **Generate embeddings**:
   ```bash
   python scripts/generate_embeddings.py
   # Uploads to Qdrant Cloud
   ```

3. **Verify**:
   ```bash
   python scripts/verify_embeddings.py
   # Expected: 320 chunks in Qdrant
   ```

---

### Phase 3: Neon Postgres Setup

1. **Create Neon account** (free tier):
   - Visit: https://neon.tech/
   - Sign up and create project

2. **Run schema migration**:
   ```bash
   cd backend
   python scripts/setup_postgres.py
   # Creates users, conversations, user_progress tables
   ```

3. **Verify**:
   ```bash
   python scripts/test_postgres.py
   ```

---

### Phase 4: Railway Deployment (Backend)

1. **Create Railway account**:
   - Visit: https://railway.app/
   - Sign up with GitHub

2. **Create new project**:
   - Connect GitHub repository
   - Select `backend/` directory as root

3. **Configure environment variables**:
   - GEMINI_API_KEY
   - COHERE_API_KEY
   - QDRANT_URL
   - QDRANT_API_KEY
   - NEON_DATABASE_URL
   - CORS_ORIGINS (include production domain)

4. **Deploy**:
   - Railway auto-deploys on git push to main
   - Monitor logs for errors

5. **Verify**:
   - Visit: https://yourapp.up.railway.app/api/health
   - Expected: `{"status": "healthy"}`

---

### Phase 5: GitHub Pages Deployment (Frontend)

1. **Update Docusaurus config**:
   ```typescript
   // book-source/docusaurus.config.ts
   const config = {
     // ...
     customFields: {
       backendUrl: 'https://yourapp.up.railway.app'
     }
   };
   ```

2. **Push to main**:
   ```bash
   git add book-source/docusaurus.config.ts
   git commit -m "feat: add backend URL for chat interface"
   git push origin main
   ```

3. **GitHub Actions auto-deploys**:
   - Workflow: `.github/workflows/deploy.yml`
   - Deploys to: https://anusbutt.github.io/hackathon-phase-01/

4. **Verify**:
   - Visit any lesson page
   - Open chat interface (bottom-right)
   - Ask a question → receive response

---

### Phase 6: Monitoring & Post-Deployment

1. **Monitor Railway logs**:
   - Check for errors, rate limit hits
   - Monitor response times

2. **Test production**:
   - Run manual QA checklist on live site
   - Ask 10 test questions
   - Verify CORS works (no console errors)

3. **Monitor API usage**:
   - Gemini: Check request count (max 1500/day)
   - Cohere: Check embedding requests (max 10,000/month)
   - Railway: Check usage credit (max $5/month)

---

## Risks & Mitigation

### Risk 1: Gemini API Rate Limit Exhaustion

**Likelihood**: Medium (15 req/min = 900/hour, but concurrent users may spike)

**Impact**: High (chatbot becomes unusable)

**Mitigation**:
- Implement request queue on backend (FIFO, max wait 30s)
- Show "Please wait, high traffic" message to users
- Monitor usage, upgrade to paid tier if needed ($7/1M tokens)

---

### Risk 2: Cohere Free Tier Exhaustion (10,000/month)

**Likelihood**: Low (only used for query embeddings, ~1 per query)

**Impact**: Medium (new queries fail, but existing embeddings still work)

**Mitigation**:
- Cache lesson metadata embeddings (static, never regenerate)
- Monitor monthly usage
- Upgrade to paid tier if needed ($1/1k embed requests)

---

### Risk 3: Qdrant Search Quality (Similarity Threshold Too Strict/Lenient)

**Likelihood**: Medium (0.7 is an educated guess, not tested)

**Impact**: Medium (poor user experience, irrelevant results or "no results found")

**Mitigation**:
- Test with 100 sample questions during development
- Adjust threshold (0.65-0.75 range) based on results
- Implement fallback: if <3 results, lower threshold to 0.6

---

### Risk 4: Browser localStorage Overflow

**Likelihood**: Low (5-10MB limit, conversations are small)

**Impact**: Low (conversation history lost, but chatbot still works)

**Mitigation**:
- Implement FIFO deletion (remove oldest conversations first)
- Warn user when approaching 80% capacity
- Provide "Clear history" button

---

### Risk 5: Railway Free Tier Exhaustion ($5/month)

**Likelihood**: Medium (depends on traffic)

**Impact**: High (backend becomes unavailable)

**Mitigation**:
- Monitor usage closely (Railway dashboard)
- Implement aggressive rate limiting (100 req/hour per user)
- Upgrade to paid tier if needed ($5-20/month)

---

### Risk 6: CORS Issues in Production

**Likelihood**: Low (well-documented, testable in staging)

**Impact**: High (frontend cannot call backend)

**Mitigation**:
- Test CORS thoroughly in staging environment
- Use Railway preview deployments for testing
- Document CORS headers clearly in API docs

---

### Risk 7: Text Selection UX on Mobile

**Likelihood**: Medium (mobile text selection is tricky)

**Impact**: Medium (feature doesn't work on mobile, but general Q&A still works)

**Mitigation**:
- Test on real mobile devices early
- Provide fallback: manual copy-paste into chat
- Consider mobile-specific tooltip positioning

---

## Open Questions

**None** - All architectural decisions documented above. Any new questions during implementation should be raised as ADRs.

---

## Success Metrics

Post-deployment (Week 1):
- [ ] 100+ user queries successfully answered
- [ ] <5% error rate (queries resulting in error messages)
- [ ] P95 response time <3s (measure via Railway logs)
- [ ] 0 security incidents (XSS, API key leaks)
- [ ] 90% user satisfaction (inferred from lack of complaints)

Post-deployment (Month 1):
- [ ] 1000+ user queries successfully answered
- [ ] <3% error rate
- [ ] Manual review confirms 95% response accuracy (100 random queries)
- [ ] 0% hallucinations (no contradictions with book content)
- [ ] Railway costs remain within $5/month (free tier)

---

## Next Steps

**After Plan Approval**:
1. Create task breakdown using `/sp.tasks` command
2. Implement in stages (infrastructure → tools → agent → frontend → integration)
3. Test after each stage (unit → integration → E2E)
4. Deploy to staging (Railway preview environment)
5. Deploy to production (Railway main + GitHub Pages)

**Estimated Timeline**:
- Stage 1 (Infrastructure): 2-3 days
- Stage 2 (Content Embedding): 1 day
- Stage 3 (Tools): 2 days
- Stage 4 (Agent): 1 day
- Stage 5 (Frontend): 2-3 days
- Stage 6 (Integration & Testing): 2-3 days
- Stage 7 (Deployment): 1 day

**Total**: ~2 weeks (assuming full-time work)

---

## Constitution Compliance Summary

✅ **All Phase 2 Constitutional Requirements Met**:

| Requirement | Compliance |
|-------------|-----------|
| LLM: Google Gemini (gemini-2.5-flash) | ✅ Decision 1 |
| Embeddings: Cohere (embed-english-v3.0) | ✅ Decision 2 |
| Vector DB: Qdrant Cloud Free Tier | ✅ Decision 3 |
| Backend: FastAPI on Railway | ✅ Architecture Overview |
| Frontend: React/TypeScript in Docusaurus | ✅ Architecture Overview |
| Anonymous Users: Browser localStorage | ✅ Decision 4 |
| Multi-turn: Last 5 messages | ✅ Decision 5 |
| Text Selection: ≥10 characters | ✅ Decision 6 |
| Similarity Threshold: 0.7 | ✅ Decision 7 |
| Rate Limiting: 100 req/hour | ✅ Decision 8 |
| Security: API keys in env vars, CORS, input sanitization | ✅ Non-Functional Requirements |
| Testing: Unit (90%), Integration, E2E | ✅ Testing Strategy |
| Error Handling: Helpful suggestions, no stack traces | ✅ Architecture Overview (Error Flow) |

**Plan Status**: READY FOR IMPLEMENTATION

---

**Document Version**: 1.0
**Last Updated**: 2025-12-30
**Author**: AI Agent (Claude Sonnet 4.5)
**Approved By**: [Pending User Approval]
