---
description: "Task list for Phase 2 RAG Chatbot implementation"
---

# Tasks: RAG Chatbot for Physical AI Book

**Input**: Design documents from `/specs/005-rag-chatbot/`
**Prerequisites**: plan.md ✅, spec.md ✅

**Tests**: Tests are included and REQUIRED per spec (90% backend coverage, E2E tests)

**Organization**: Tasks are grouped by implementation stage to enable incremental delivery. P1 user stories (general Q&A + selected text) are prioritized for MVP.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1=General Q&A, US2=Selected Text, US3=Multi-turn, etc.)
- Exact file paths included in descriptions

## Path Conventions

- **Backend**: `backend/app/`, `backend/tests/`, `backend/scripts/`
- **Frontend**: `book-source/src/components/`, `book-source/src/hooks/`, `book-source/src/services/`

---

## Phase 0: Project Setup

**Purpose**: Initialize Phase 2 backend infrastructure and development environment

**Prerequisites**: None (can start immediately)

### Backend Structure

- [ ] **T001** [P] Create backend directory structure
  - `backend/app/{__init__.py, main.py}`
  - `backend/app/{config,models,agents,tools,services,routers,middleware}/`
  - `backend/scripts/`
  - `backend/tests/{unit,integration,e2e}/`
  - `backend/.env.example`
  - `backend/requirements.txt`
  - `backend/requirements-dev.txt`

- [ ] **T002** [P] Initialize Python dependencies in `backend/requirements.txt`
  - FastAPI >=0.100.0
  - agents (OpenAI Agents SDK)
  - openai (AsyncOpenAI client)
  - cohere
  - qdrant-client
  - asyncpg
  - pydantic >=2.0
  - python-dotenv
  - uvicorn[standard]

- [ ] **T003** [P] Initialize dev dependencies in `backend/requirements-dev.txt`
  - pytest
  - pytest-asyncio
  - httpx (for async API tests)
  - black (code formatter)
  - ruff (linter)
  - mypy (type checker)

- [ ] **T004** Create `.env.example` with required environment variables
  - GEMINI_API_KEY, GEMINI_BASE_URL, GEMINI_MODEL
  - COHERE_API_KEY, COHERE_EMBEDDING_MODEL
  - QDRANT_URL, QDRANT_API_KEY, QDRANT_COLLECTION_NAME
  - NEON_DATABASE_URL
  - CORS_ORIGINS, API_RATE_LIMIT

- [ ] **T005** [P] Update `.gitignore` with Phase 2 additions
  - `backend/.env`
  - `backend/__pycache__/`
  - `backend/.pytest_cache/`
  - `backend/venv/`
  - `*.pyc`

### Local Development Setup

- [ ] **T006** Create FastAPI application entry point `backend/app/main.py`
  - Initialize FastAPI app
  - Add CORS middleware
  - Add basic health check endpoint (`GET /api/health`)
  - Configure logging

- [ ] **T007** [P] Create settings module `backend/app/config/settings.py`
  - Load environment variables with Pydantic BaseSettings
  - Validate required API keys on startup
  - Export settings instance for app-wide use

- [ ] **T008** Test local backend startup
  - Run `uvicorn app.main:app --reload`
  - Verify `http://localhost:8000/api/health` returns 200
  - Verify CORS headers present

**Checkpoint**: Backend project structure initialized, can run locally

---

## Phase 1: Foundational Infrastructure

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### External Services Setup

- [ ] **T009** Setup Qdrant Cloud Free Tier account
  - Create account at https://cloud.qdrant.io/
  - Create cluster (free tier, 1GB)
  - Get API key and cluster URL
  - Add to `.env`: QDRANT_URL, QDRANT_API_KEY

- [ ] **T010** Create Qdrant collection `backend/scripts/setup_qdrant.py`
  - Collection name: `humanoid_robotics_book`
  - Vector size: 1024 (Cohere embed-english-v3.0)
  - Distance: Cosine
  - Create collection with metadata schema (module, lesson, section, tags, skills, bloom_level)

- [ ] **T011** Test Qdrant connection `backend/scripts/test_qdrant.py`
  - Connect to Qdrant Cloud
  - Verify collection exists
  - Insert test vector, search test vector
  - Delete test data

- [ ] **T012** Setup Neon Serverless Postgres account
  - Create account at https://neon.tech/
  - Create project (free tier)
  - Get connection string
  - Add to `.env`: NEON_DATABASE_URL

- [ ] **T013** Create Postgres schema `backend/scripts/setup_postgres.py`
  - Create `users` table (id, username, email, created_at, current_module, current_lesson, preferences)
  - Create `conversations` table (id, user_id, query, response, selected_text, lesson_id, sources, created_at, rating)
  - Create `user_progress` table (id, user_id, module, lesson, completed, time_spent_seconds, quiz_score, last_accessed)
  - Add indexes (user_id, created_at)

- [ ] **T014** Test Postgres connection `backend/scripts/test_postgres.py`
  - Connect to Neon Postgres
  - Insert test user, retrieve user
  - Delete test data

- [ ] **T015** Get API keys for external services
  - Google Gemini API key (https://aistudio.google.com/app/apikey)
  - Cohere API key (https://dashboard.cohere.com/api-keys)
  - Add to `.env`: GEMINI_API_KEY, COHERE_API_KEY

### Core Services Implementation

- [ ] **T016** [P] Implement Qdrant service `backend/app/services/qdrant_service.py`
  - Initialize QdrantClient with URL and API key
  - Method: `search(query_vector, limit, score_threshold, filters)` → list[SearchResult]
  - Method: `insert_chunks(chunks)` → success/failure
  - Handle connection errors gracefully

- [ ] **T017** [P] Implement Cohere service `backend/app/services/cohere_service.py`
  - Initialize Cohere client with API key
  - Method: `embed_query(text)` → list[float] (1024 dims)
  - Method: `embed_documents(texts)` → list[list[float]]
  - Use `input_type="search_query"` for queries, `"search_document"` for indexing

- [ ] **T018** [P] Implement Postgres service `backend/app/services/postgres_service.py`
  - Create async connection pool with asyncpg
  - Method: `get_connection()` → async context manager
  - Method: `execute(query, params)` → result
  - Handle connection errors gracefully

- [ ] **T019** [P] Create Pydantic models `backend/app/models/schemas.py`
  - `ChatQueryRequest(query, selected_text?, lesson_id?, conversation_history)`
  - `ChatQueryResponse(response, sources, conversation_id, timestamp)`
  - `ChatFeedbackRequest(conversation_id, rating, comment?)`
  - `HealthResponse(status, services, timestamp)`
  - `Message(role, content, timestamp, sources?, selected_text?)`
  - `Source(module, lesson, section?)`

- [ ] **T020** [P] Create domain entities `backend/app/models/entities.py`
  - `ContentChunk(chunk_id, content, embedding, metadata)`
  - `ChunkMetadata(module, lesson, section, tags, skills, bloom_level, lesson_title, module_title)`
  - `Conversation(conversation_id, messages, created_at, expires_at)`

**Checkpoint**: All external services configured and testable, core services implemented

---

## Phase 2: Content Embedding Pipeline (One-Time Setup)

**Purpose**: Extract Phase 1 book content and generate embeddings for Qdrant

**Prerequisites**: Phase 1 complete (Qdrant + Cohere configured)

### Content Extraction

- [ ] **T021** Implement lesson extraction script `backend/scripts/extract_lessons.py`
  - Read all MDX files from `book-source/docs/13-Physical-AI-Humanoid-Robotics/`
  - Parse frontmatter (title, skills, learning_objectives, tags)
  - Extract content, split by H2/H3 sections
  - Output: `backend/data/extracted_lessons.json`

- [ ] **T022** Implement lesson metadata generator `backend/scripts/generate_metadata.py`
  - Parse all lesson frontmatter
  - Create metadata JSON mapping lesson_id → metadata
  - Output: `backend/data/lesson_metadata.json`

- [ ] **T023** Test extraction on 1 module
  - Run `extract_lessons.py` on Module 1 only
  - Verify frontmatter parsed correctly
  - Verify sections split correctly (H2/H3 boundaries)

### Embedding Generation

- [ ] **T024** Implement chunking strategy `backend/scripts/chunk_content.py`
  - Input: extracted_lessons.json
  - Split content into 500-1000 token chunks
  - Preserve H2/H3 section boundaries (don't split mid-section)
  - Add metadata to each chunk (module, lesson, section, tags, skills)
  - Output: `backend/data/chunks.json`

- [ ] **T025** Implement embedding generation script `backend/scripts/generate_embeddings.py`
  - Input: chunks.json
  - For each chunk:
    - Generate embedding via Cohere (`embed_documents`)
    - Create PointStruct with chunk_id, embedding, payload (content + metadata)
  - Batch process (100 chunks per batch)
  - Upload to Qdrant collection `humanoid_robotics_book`
  - Log progress (X/Y chunks processed)

- [ ] **T026** Run embedding pipeline on all 4 modules
  - Extract all lessons → chunk content → generate embeddings → upload to Qdrant
  - Verify ~320 chunks in Qdrant (16 lessons × ~20 chunks/lesson)
  - Expected time: ~10 minutes

- [ ] **T027** Verify embeddings in Qdrant
  - Test query: "What is ROS2?"
  - Generate query embedding → search Qdrant → verify top results are from Module 1
  - Check similarity scores (should be >0.7 for relevant content)

**Checkpoint**: All book content embedded and searchable in Qdrant

---

## Phase 3: User Story 1 - Ask General Questions (Priority: P1) 🎯 MVP

**Goal**: Students can ask questions about book content and receive answers with source citations

**Independent Test**: Ask "What is ROS2?" → Receive response citing Module 1: ROS2 Nervous System

### Unit Tests for User Story 1

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] **T028** [P] [US1] Unit test for search_book_content tool `backend/tests/unit/test_tools.py::test_search_book_content`
  - Mock Qdrant search
  - Mock Cohere embedding
  - Verify tool returns expected format (content, score, module, lesson)
  - Test with module_filter
  - Test with empty results (similarity <0.7)

- [ ] **T029** [P] [US1] Unit test for Qdrant service `backend/tests/unit/test_services.py::test_qdrant_search`
  - Mock QdrantClient
  - Test search with filters
  - Test connection error handling

- [ ] **T030** [P] [US1] Unit test for Cohere service `backend/tests/unit/test_services.py::test_cohere_embed_query`
  - Mock Cohere client
  - Test query embedding generation
  - Test batch document embedding

### Implementation for User Story 1

- [ ] **T031** [P] [US1] Implement search_book_content tool `backend/app/tools/search_content.py`
  - `@function_tool` decorator
  - Input: query (str), module_filter (str | None), top_k (int)
  - Generate query embedding via Cohere
  - Search Qdrant with filters, score_threshold=0.7, limit=top_k
  - Format results: [{content, score, module, lesson, section}, ...]
  - Return empty list if no results (trigger error handling)

- [ ] **T032** [US1] Initialize Gemini LLM with OpenAI Agents SDK `backend/app/agents/chatbot_agent.py`
  - Import: `from agents import Agent, Runner, OpenAIChatCompletionsModel, AsyncOpenAI`
  - Create AsyncOpenAI client with GEMINI_API_KEY and GEMINI_BASE_URL
  - Create OpenAIChatCompletionsModel with model="gemini-2.5-flash"
  - Define system prompt (educational tone, cite sources, accurate)

- [ ] **T033** [US1] Create agent with search_book_content tool `backend/app/agents/chatbot_agent.py`
  - Initialize Agent with name="RAG_Assistant", model=llm_model, tools=[search_book_content]
  - Method: `process_query(query, conversation_history)` → response
  - Include last 5 messages in conversation_history

- [ ] **T034** [US1] Implement POST /api/chat/query endpoint `backend/app/routers/chat.py`
  - Accept ChatQueryRequest (query, conversation_history)
  - Validate query length (<2000 chars)
  - Initialize agent
  - Call `Runner.run_sync(starting_agent=agent, input=query)`
  - Extract response and sources from agent output
  - Return ChatQueryResponse

- [ ] **T035** [US1] Add CORS middleware `backend/app/middleware/cors.py`
  - Allow origins: `https://anusbutt.github.io`, `http://localhost:3000`
  - Allow methods: POST, GET, OPTIONS
  - Allow headers: Content-Type

- [ ] **T036** [US1] Add error handling middleware `backend/app/middleware/error_handler.py`
  - Catch all exceptions
  - Return user-friendly error messages (no stack traces)
  - Log errors to Railway logs
  - Return HTTP 503 for external API failures (Gemini, Qdrant, Cohere)

### Integration Tests for User Story 1

- [ ] **T037** [P] [US1] Integration test for chat endpoint `backend/tests/integration/test_api.py::test_chat_query_success`
  - Use TestClient (httpx)
  - POST /api/chat/query with real Qdrant + real Gemini
  - Verify response contains "response", "sources", "conversation_id"
  - Verify sources[0] has module and lesson fields

- [ ] **T038** [P] [US1] Integration test for agent with tool `backend/tests/integration/test_agent.py::test_agent_calls_search_tool`
  - Initialize agent with search_book_content
  - Provide query "What is ROS2?"
  - Verify agent calls search_book_content tool
  - Verify agent generates response from tool output

**Checkpoint**: User Story 1 complete - general Q&A works end-to-end

---

## Phase 4: User Story 2 - Ask About Selected Text (Priority: P1) 🎯 MVP

**Goal**: Students can select text on lesson page and ask questions about it with contextual awareness

**Independent Test**: Select "DDS middleware" → Ask "What is this?" → Receive response prioritizing selected text context

### Unit Tests for User Story 2

- [ ] **T039** [P] [US2] Unit test for get_selected_text_context tool `backend/tests/unit/test_tools.py::test_get_selected_text_context`
  - Mock Qdrant search with filters
  - Verify tool finds chunk containing selected_text
  - Verify surrounding_context includes paragraphs before/after
  - Test when selected_text not found (fallback behavior)

### Implementation for User Story 2

- [ ] **T040** [P] [US2] Implement get_selected_text_context tool `backend/app/tools/selected_text.py`
  - `@function_tool` decorator
  - Input: selected_text (str), lesson_id (str)
  - Parse lesson_id → extract module, lesson
  - Query Qdrant with metadata filter (module=X, lesson=Y)
  - Find chunk containing selected_text
  - Extract surrounding paragraphs (1 before, 1 after)
  - Return: {selected_text, surrounding_context, lesson_metadata}

- [ ] **T041** [US2] Update chatbot agent with selected_text tool `backend/app/agents/chatbot_agent.py`
  - Add get_selected_text_context to tools list
  - Update system prompt: "When selected_text is provided, prioritize it over general search"

- [ ] **T042** [US2] Update POST /api/chat/query for selected text `backend/app/routers/chat.py`
  - Accept selected_text and lesson_id in ChatQueryRequest
  - Pass selected_text context to agent
  - Agent automatically calls get_selected_text_context when selected_text present

### Integration Tests for User Story 2

- [ ] **T043** [P] [US2] Integration test for selected text query `backend/tests/integration/test_api.py::test_chat_query_with_selected_text`
  - POST /api/chat/query with selected_text="DDS middleware" and lesson_id="module-01/lesson-02"
  - Verify response mentions DDS
  - Verify sources include Module 1 Lesson 2

**Checkpoint**: User Story 2 complete - selected text Q&A works

---

## Phase 5: User Story 3 - Multi-Turn Conversations (Priority: P2)

**Goal**: Students can ask follow-up questions and chatbot maintains context of last 5 messages

**Independent Test**: Ask "What is ROS2?" → Ask "How is it different from ROS1?" → Verify chatbot references "ROS2" from previous message

### Implementation for User Story 3

- [ ] **T044** [US3] Implement conversation context preparation `backend/app/agents/chatbot_agent.py`
  - Method: `prepare_conversation_context(conversation_history)` → str
  - Extract last 5 messages
  - Format as "USER: ...\nASSISTANT: ...\n"
  - Include in agent's context

- [ ] **T045** [US3] Update agent to include conversation history `backend/app/agents/chatbot_agent.py`
  - When processing query, prepend conversation_history to agent context
  - Agent uses history to resolve pronouns ("it", "that", "which")

### Integration Tests for User Story 3

- [ ] **T046** [P] [US3] Integration test for multi-turn conversation `backend/tests/integration/test_api.py::test_multi_turn_conversation`
  - POST query 1: "What is ROS2?"
  - Save response as message 1
  - POST query 2: "How is it different from ROS1?" with conversation_history=[message 1]
  - Verify response references ROS2 (context maintained)

**Checkpoint**: User Story 3 complete - multi-turn conversations work

---

## Phase 6: User Story 5 - Error Handling & Suggestions (Priority: P3)

**Goal**: When queries fail or are out-of-scope, provide helpful error messages and suggestions

**Independent Test**: Ask "What's the weather?" → Receive "Outside book scope. I can help with: ROS2, sensors, Isaac, VLA"

### Implementation for User Story 5

- [ ] **T047** [US5] Implement error handling for no results `backend/app/tools/search_content.py`
  - When Qdrant returns 0 results (similarity <0.7 for all chunks):
  - Return suggestion: "I couldn't find specific information about that in the book. Could you rephrase your question or try asking about: [suggested topics from metadata]"

- [ ] **T048** [US5] Implement out-of-scope detection `backend/app/agents/chatbot_agent.py`
  - Add system prompt instruction: "If question is outside book scope (installation, weather, non-robotics), politely decline and suggest book topics"
  - Example: "That question is outside the scope of this Physical AI and Humanoid Robotics book. I can help you with questions about ROS2, sensors, NVIDIA Isaac, or vision-language-action models."

- [ ] **T049** [US5] Add API error handling `backend/app/middleware/error_handler.py`
  - Catch QdrantException → Return HTTP 503: "Search is temporarily unavailable"
  - Catch CohereException → Return HTTP 503: "Embedding service unavailable"
  - Catch Gemini API errors → Return HTTP 429: "Too many requests. Please wait." or HTTP 503: "Chatbot temporarily unavailable"

### Integration Tests for User Story 5

- [ ] **T050** [P] [US5] Integration test for out-of-scope query `backend/tests/integration/test_api.py::test_out_of_scope_query`
  - POST query: "What's the weather today?"
  - Verify response contains "outside the scope of this book"

- [ ] **T051** [P] [US5] Integration test for no results `backend/tests/integration/test_api.py::test_no_results_found`
  - POST query with obscure term (e.g., "quantum entanglement robotics")
  - Verify response suggests rephrasing or alternative topics

**Checkpoint**: User Story 5 complete - helpful error handling works

---

## Phase 7: User Story 4 - Cross-Lesson Knowledge (Priority: P2)

**Goal**: Chatbot retrieves and synthesizes information from multiple modules

**Independent Test**: Ask "How do VLAs use ROS2?" → Receive answer citing both Module 1 (ROS2) and Module 4 (VLA)

### Implementation for User Story 4

- [ ] **T052** [US4] Verify cross-module search works `backend/app/tools/search_content.py`
  - No code changes needed (already searches across all modules by default)
  - Ensure no module_filter applied unless explicitly requested

### Integration Tests for User Story 4

- [ ] **T053** [P] [US4] Integration test for cross-module query `backend/tests/integration/test_api.py::test_cross_module_query`
  - POST query: "How do VLAs use sensors?"
  - Verify sources include both Module 2 (Sensors) and Module 4 (VLA)

**Checkpoint**: User Story 4 complete - cross-module retrieval works

---

## Phase 8: Backend Deployment to Railway

**Purpose**: Deploy backend to production Railway environment

**Prerequisites**: Phases 3-7 complete (all P1-P2 user stories implemented)

### Railway Setup

- [ ] **T054** Create Railway account and project
  - Sign up at https://railway.app/ with GitHub
  - Create new project
  - Connect GitHub repository `anusbutt/hackathon-phase-01`
  - Set root directory to `backend/`

- [ ] **T055** Configure Railway environment variables
  - Add all variables from `.env.example`
  - GEMINI_API_KEY, COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL
  - CORS_ORIGINS=https://anusbutt.github.io,http://localhost:3000
  - API_RATE_LIMIT=100

- [ ] **T056** Create railway.toml configuration `backend/railway.toml`
  - Build command: `pip install -r requirements.txt`
  - Start command: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
  - Python version: 3.11

- [ ] **T057** Deploy to Railway
  - Push to main branch → Railway auto-deploys
  - Monitor deployment logs for errors
  - Get Railway URL (e.g., `https://hackathon-phase-01.up.railway.app`)

- [ ] **T058** Verify Railway deployment
  - Visit `https://yourapp.up.railway.app/api/health`
  - Expected: `{"status": "healthy", "services": {"qdrant": true, "gemini": true, "cohere": true}}`
  - Test chat endpoint with curl: `curl -X POST https://yourapp.up.railway.app/api/chat/query -H "Content-Type: application/json" -d '{"query": "What is ROS2?"}'`

**Checkpoint**: Backend deployed to Railway and accessible via public URL

---

## Phase 9: Frontend Components (React/TypeScript)

**Purpose**: Add chat interface components to existing Docusaurus site

**Prerequisites**: Phase 8 complete (backend deployed, URL available)

### Frontend Infrastructure

- [ ] **T059** [P] Install frontend dependencies `book-source/package.json`
  - Add dependencies: (most already exist in Docusaurus)
  - Check if any additional libraries needed for chat UI

- [ ] **T060** [P] Create TypeScript types `book-source/src/types/chat.ts`
  - `interface Message { role: "user" | "assistant", content: string, timestamp: string, sources?: Source[], selected_text?: string }`
  - `interface Source { module: string, lesson: string, section?: string }`
  - `interface Conversation { conversation_id: string, messages: Message[], created_at: string, expires_at: string }`

- [ ] **T061** [P] Create API client `book-source/src/services/chatApi.ts`
  - Method: `sendQuery(query, selectedText?, lessonId?, conversationHistory?) → Promise<ChatQueryResponse>`
  - Method: `sendFeedback(conversationId, rating, comment?) → Promise<void>`
  - Use fetch API, target Railway backend URL (from docusaurus.config.ts customFields)
  - Handle errors (network, 429, 503)

### User Story 1 & 2: Chat Interface + Text Selection (P1)

- [ ] **T062** [US1] [US2] Create ChatInterface component `book-source/src/components/ChatInterface.tsx`
  - Fixed bottom-right floating widget
  - Expand/collapse button
  - Message list (user + assistant messages)
  - Input field + send button
  - Loading indicator (typing animation)
  - Source citations display
  - Dark/light theme support

- [ ] **T063** [US1] [US2] Create ChatMessage component `book-source/src/components/ChatMessage.tsx`
  - Props: message (Message)
  - Display user messages (right-aligned, different color)
  - Display assistant messages (left-aligned, with sources)
  - Format sources: "Source: Module X: Module Name - Lesson Y"

- [ ] **T064** [US1] [US2] Create ChatInput component `book-source/src/components/ChatInput.tsx`
  - Textarea for query input
  - Send button
  - Disable during loading
  - Validate input (non-empty, <2000 chars)
  - Handle Enter key (send), Shift+Enter (new line)

- [ ] **T065** [US2] Create TextSelectionHandler component `book-source/src/components/TextSelectionHandler.tsx`
  - Listen to mouseup event on lesson pages
  - Detect selected text (window.getSelection())
  - Show tooltip if selection ≥10 characters
  - Tooltip has "Ask about this" button
  - Clicking button: opens chat, prefills selected text context

### User Story 3 & 7: Conversation History (P2-P3)

- [ ] **T066** [US3] [US7] Create useChatState hook `book-source/src/hooks/useChatState.ts`
  - State: messages (Message[]), isOpen (boolean), isLoading (boolean)
  - Actions: sendMessage, toggleChat, clearHistory
  - Call chatApi.sendQuery when sending message
  - Update messages with response

- [ ] **T067** [US3] [US7] Create useConversationHistory hook `book-source/src/hooks/useConversationHistory.ts`
  - Load conversation from localStorage on mount
  - Save conversation to localStorage on message add
  - Check expiry (7 days), clear if expired
  - Implement FIFO deletion if localStorage approaching limit
  - Return: messages, saveMessage, clearHistory

- [ ] **T068** [US2] Create useTextSelection hook `book-source/src/hooks/useTextSelection.ts`
  - State: selectedText, lessonId, showTooltip, tooltipPosition
  - Listen to mouseup event
  - Extract selected text, get lesson ID from URL
  - Calculate tooltip position near selection
  - Return: selectedText, showTooltip, tooltipPosition, hideTooltip

### Docusaurus Integration

- [ ] **T069** Update docusaurus.config.ts with backend URL `book-source/docusaurus.config.ts`
  - Add to customFields: `backendUrl: 'https://yourapp.up.railway.app'`
  - Access in components via `useDocusaurusContext()`

- [ ] **T070** Add ChatInterface to Docusaurus layout `book-source/src/theme/Root.tsx`
  - Create `src/theme/Root.tsx` (swizzle Docusaurus Root component)
  - Render ChatInterface globally (available on all pages)
  - Render TextSelectionHandler on lesson pages only

- [ ] **T071** [P] Create chat component styles `book-source/src/css/chat.module.css`
  - Floating widget styles (bottom-right position)
  - Message bubbles (user vs assistant)
  - Input field, send button
  - Tooltip styles
  - Dark mode support (CSS variables)

**Checkpoint**: Frontend components implemented, integrated into Docusaurus

---

## Phase 10: End-to-End Testing

**Purpose**: Test complete chat flow from frontend → backend → response

**Prerequisites**: Phase 9 complete (frontend deployed)

### E2E Tests (Playwright)

- [ ] **T072** [P] Setup Playwright `book-source/package.json`
  - Add dev dependency: `@playwright/test`
  - Create `book-source/tests/e2e/` directory
  - Create `playwright.config.ts`

- [ ] **T073** [US1] E2E test for general Q&A `book-source/tests/e2e/chat.spec.ts::test_general_question`
  - Navigate to lesson page
  - Open chat interface (click toggle)
  - Type "What is ROS2?" and click send
  - Wait for response (max 5 seconds)
  - Verify response contains "ROS2"
  - Verify source citation visible

- [ ] **T074** [US2] E2E test for selected text `book-source/tests/e2e/chat.spec.ts::test_selected_text_question`
  - Navigate to Module 1 Lesson 2
  - Select text "DDS middleware" (use Playwright page.locator().selectText())
  - Click "Ask about this" tooltip button
  - Verify chat opens with selected text context
  - Type "What is this?" and send
  - Verify response mentions DDS

- [ ] **T075** [US3] E2E test for multi-turn conversation `book-source/tests/e2e/chat.spec.ts::test_multi_turn_conversation`
  - Ask "What is ROS2?"
  - Wait for response
  - Ask "How is it different from ROS1?"
  - Verify response references ROS2 (context maintained)

- [ ] **T076** [US5] E2E test for error handling `book-source/tests/e2e/chat.spec.ts::test_error_message`
  - Ask "What's the weather today?"
  - Verify response contains "outside the scope of this book"

- [ ] **T077** Run E2E tests on production site
  - Set baseURL to `https://anusbutt.github.io/hackathon-phase-01/`
  - Run all E2E tests
  - Verify all pass

**Checkpoint**: E2E tests pass, full chat flow working

---

## Phase 11: Additional Features (P3)

**Purpose**: Polish features from User Story 6 (visual feedback) and remaining tools

### Visual Polish (User Story 6)

- [ ] **T078** [US6] Add typing indicator to ChatInterface `book-source/src/components/ChatInterface.tsx`
  - Show animated dots while waiting for response
  - Display "Assistant is typing..." message

- [ ] **T079** [US6] Add streaming response support (optional, if time allows)
  - Backend: Use SSE (Server-Sent Events) for streaming responses
  - Frontend: Display response word-by-word as it arrives
  - Fallback to standard POST if streaming fails

- [ ] **T080** [US6] Add smooth animations `book-source/src/css/chat.module.css`
  - Fade in new messages
  - Expand/collapse chat widget animation
  - Tooltip fade in/out

### Remaining Tools (Phase 2 Future-Proofing)

- [ ] **T081** [P] Implement get_lesson_metadata tool `backend/app/tools/lesson_metadata.py`
  - Load metadata from `backend/data/lesson_metadata.json`
  - Cache in memory (metadata is static)
  - Return metadata for given lesson_id

- [ ] **T082** [P] Implement get_user_profile tool `backend/app/tools/user_profile.py`
  - Phase 2: Always return None (anonymous users only)
  - Add TODO comment for Phase 3 implementation (Neon Postgres query)

- [ ] **T083** [P] Implement save_conversation tool `backend/app/tools/conversation.py`
  - Phase 2: Return random UUID (no database write)
  - Add TODO comment for Phase 3 implementation (Neon Postgres insert)

**Checkpoint**: All P3 features complete

---

## Phase 12: Rate Limiting & Security

**Purpose**: Implement rate limiting and security hardening

### Rate Limiting

- [ ] **T084** Implement rate limiting middleware `backend/app/middleware/rate_limit.py`
  - Track requests by session ID (hash of User-Agent + IP)
  - Limit: 100 requests/hour per session
  - Return HTTP 429 with "Retry-After: 3600" header when exceeded
  - Clean expired entries periodically

- [ ] **T085** Add rate limit to chat endpoint `backend/app/routers/chat.py`
  - Apply rate_limit_middleware to POST /api/chat/query
  - Test: Send 101 requests in 1 hour → 101st returns 429

### Security Hardening

- [ ] **T086** Input sanitization for query `backend/app/routers/chat.py`
  - Pydantic already validates query length (<2000 chars)
  - Add HTML escaping (prevent XSS in responses)
  - Verify no SQL injection vectors (using parameterized queries in Postgres)

- [ ] **T087** Verify CORS configuration `backend/app/middleware/cors.py`
  - Allowed origins: Only `https://anusbutt.github.io` and `http://localhost:3000`
  - No wildcard "*" allowed
  - Test CORS with curl from unauthorized origin → Should block

- [ ] **T088** Verify secrets not in git `backend/.env`
  - Run: `git log --all --full-history -- "*/.env"`
  - Expected: No results (no .env files ever committed)
  - Verify `.env` in `.gitignore`

**Checkpoint**: Security hardening complete, rate limiting active

---

## Phase 13: Documentation & Deployment

**Purpose**: Document deployment process, update README, deploy to production

### Documentation

- [ ] **T089** Create deployment guide `specs/005-rag-chatbot/deployment.md`
  - Backend deployment to Railway (step-by-step)
  - Environment variable configuration
  - Frontend update (add backend URL)
  - Embedding pipeline (one-time setup)
  - Troubleshooting common issues

- [ ] **T090** Update main README `README.md`
  - Add Phase 2 section
  - Link to deployed chatbot
  - Document features (general Q&A, selected text, multi-turn)
  - Link to API documentation (FastAPI /docs endpoint)

- [ ] **T091** Create API documentation `backend/README.md`
  - Link to auto-generated Swagger docs at `/docs`
  - Document rate limits, CORS policy
  - Example curl commands for testing

### Final Deployment

- [ ] **T092** Deploy frontend to GitHub Pages
  - Update `book-source/docusaurus.config.ts` with production backend URL
  - Commit and push to main
  - GitHub Actions auto-deploys
  - Verify: `https://anusbutt.github.io/hackathon-phase-01/`

- [ ] **T093** Verify production deployment
  - Visit lesson page
  - Open chat interface
  - Ask "What is ROS2?"
  - Verify response appears with sources
  - Test selected text feature
  - Test multi-turn conversation (3+ messages)

- [ ] **T094** Monitor Railway logs
  - Check for errors, warnings
  - Verify request counts (should be <1500/day for Gemini free tier)
  - Monitor Cohere usage (should be <10,000/month)

**Checkpoint**: Phase 2 complete, deployed to production

---

## Phase 14: Testing & Quality Assurance

**Purpose**: Manual QA, performance testing, security audit

### Manual QA Checklist

- [ ] **T095** Test 10 sample questions across all 4 modules
  - Module 1: "What is ROS2?", "Explain pub/sub model"
  - Module 2: "What sensors are used in humanoid robots?", "Explain lidar vs depth cameras"
  - Module 3: "What is NVIDIA Isaac Sim?", "How does Isaac ROS work?"
  - Module 4: "What are vision-language-action models?", "How do VLAs control robots?"
  - Verify all responses accurate, cite correct sources

- [ ] **T096** Test selected text feature on 5 different lessons
  - Select text in Module 1 Lesson 2, ask question
  - Select code block in Module 2 capstone project, ask question
  - Verify responses prioritize selected text context

- [ ] **T097** Test multi-turn conversation (5+ messages)
  - Ask follow-up questions with pronouns ("it", "that", "which one")
  - Verify chatbot resolves pronouns correctly using history

- [ ] **T098** Test error handling
  - Ask out-of-scope question → Verify helpful error message
  - Simulate Qdrant down (disconnect) → Verify graceful error
  - Send 101 requests in 1 hour → Verify rate limit message

- [ ] **T099** Test browser compatibility
  - Chrome (latest)
  - Firefox (latest)
  - Safari (latest)
  - Edge (latest)
  - Verify chat interface works on all browsers

- [ ] **T100** Test mobile responsiveness
  - iPhone (Safari)
  - Android (Chrome)
  - Verify text selection works on mobile
  - Verify chat interface responsive (small screens)

- [ ] **T101** Test dark mode
  - Switch Docusaurus theme to dark mode
  - Verify chat interface adapts (colors, contrast)

### Performance Testing

- [ ] **T102** Measure response times (P95)
  - Send 100 queries, record response times
  - Calculate P95 latency
  - Expected: <3 seconds for 95% of queries

- [ ] **T103** Load testing (concurrent users)
  - Use tool (e.g., Apache Bench, Locust) to simulate 50 concurrent users
  - Each user sends 10 queries
  - Verify no errors, response times acceptable

### Security Audit

- [ ] **T104** Manual security review
  - Verify all API keys in Railway environment variables (not in git)
  - Verify CORS restricts to production domain only
  - Test XSS: Submit query with `<script>alert('xss')</script>` → Should be escaped
  - Test SQL injection (if any direct SQL): Submit query with `'; DROP TABLE users;--` → Should be sanitized

- [ ] **T105** Run dependency security scan
  - Backend: `pip audit` or `safety check`
  - Frontend: `npm audit`
  - Fix any high/critical vulnerabilities

**Checkpoint**: QA complete, no blocking issues

---

## Dependencies & Execution Order

### Phase Dependencies

**Sequential Phases** (must complete in order):
1. **Phase 0: Setup** → Phase 1
2. **Phase 1: Foundational** → Phase 2
3. **Phase 2: Embedding Pipeline** → Phase 3-7 (user stories)
4. **Phase 3-7: User Stories** → Phase 8 (backend deployment)
5. **Phase 8: Backend Deployment** → Phase 9 (frontend)
6. **Phase 9: Frontend** → Phase 10 (E2E tests)
7. **Phase 10: E2E Tests** → Phase 11-13 (polish, docs, final deployment)

**Parallel Opportunities**:
- **Phase 1**: T016-T020 (services + models) can run in parallel (marked [P])
- **Phase 3-7**: User stories can be implemented in parallel by different developers (after Phase 2 complete)
- **Phase 9**: Frontend components T062-T071 can be built in parallel (marked [P])
- **Phase 14**: QA tasks T095-T101 can run in parallel

### Critical Path (MVP - P1 Stories Only)

Fastest path to working chatbot:
1. Phase 0: Setup (T001-T008) - **1 day**
2. Phase 1: Foundational (T009-T020) - **2 days**
3. Phase 2: Embedding Pipeline (T021-T027) - **1 day**
4. Phase 3: User Story 1 (T028-T038) - **2 days**
5. Phase 4: User Story 2 (T039-T043) - **1 day**
6. Phase 8: Backend Deployment (T054-T058) - **0.5 day**
7. Phase 9: Frontend (T059-T071) - **2 days**
8. Phase 10: E2E Tests (T072-T077) - **1 day**
9. Phase 13: Final Deployment (T092-T094) - **0.5 day**

**Total MVP (P1)**: ~11 days

### Full Feature Path (P1 + P2 + P3)

Add after MVP:
- Phase 5: User Story 3 (T044-T046) - **1 day**
- Phase 6: User Story 5 (T047-T051) - **1 day**
- Phase 7: User Story 4 (T052-T053) - **0.5 day**
- Phase 11: Polish (T078-T083) - **1 day**
- Phase 12: Security (T084-T088) - **1 day**

**Total Full Feature**: ~16 days

---

## Implementation Strategy

### Recommended Approach: Incremental MVP

**Week 1: Foundation + Embedding**
- Days 1-2: Phase 0-1 (Setup + Foundational)
- Day 3: Phase 2 (Embedding Pipeline)
- **Checkpoint**: Qdrant populated, backend skeleton ready

**Week 2: Core Features (P1)**
- Days 4-5: Phase 3 (User Story 1 - General Q&A)
- Day 6: Phase 4 (User Story 2 - Selected Text)
- **Checkpoint**: P1 stories complete (backend only)

**Week 3: Deployment + Frontend**
- Day 7: Phase 8 (Backend Deployment to Railway)
- Days 8-9: Phase 9 (Frontend Components)
- Day 10: Phase 10 (E2E Tests)
- **Checkpoint**: MVP deployed to production

**Week 4: Polish + Full Features (P2-P3)**
- Day 11: Phase 5-7 (User Stories 3, 4, 5)
- Day 12: Phase 11 (Polish)
- Day 13: Phase 12 (Security)
- Day 14: Phase 13-14 (Docs + QA)

**Total**: ~3 weeks (15 work days)

---

## Success Metrics

**MVP Launch (After Week 3)**:
- [ ] 100+ test queries answered successfully
- [ ] <5% error rate
- [ ] P95 response time <3s
- [ ] Chat interface visible on all lesson pages
- [ ] Selected text feature works on desktop + mobile

**Full Feature Launch (After Week 4)**:
- [ ] All P1-P3 user stories complete
- [ ] Manual QA passes (100 test queries)
- [ ] 95% response accuracy (manual review)
- [ ] 0% hallucinations (no contradictions with book content)
- [ ] Railway costs <$5/month (free tier)
- [ ] Documentation complete (README, deployment guide, API docs)

---

## Notes

- **[P] tasks**: Can run in parallel (different files, no dependencies)
- **[Story] label**: Maps task to user story (US1, US2, US3, etc.)
- **Tests FIRST**: Write unit/integration tests before implementation (they should fail initially)
- **Commit frequently**: After each task or logical group
- **Checkpoints**: Stop and validate independently before proceeding
- **Environment**: Always use `.env` for secrets (never commit)
- **Railway**: Monitor logs during deployment for early error detection

---

**Total Tasks**: 105
**Estimated Timeline**: 15-20 work days (3-4 weeks)
**MVP Timeline**: 11 days (2 weeks)

**Status**: Ready for implementation ✅
