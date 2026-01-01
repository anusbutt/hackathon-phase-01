# Feature Specification: RAG Chatbot for Physical AI Book

**Feature Branch**: `005-rag-chatbot`
**Created**: 2025-12-30
**Status**: Draft
**Input**: Build and embed a Retrieval-Augmented Generation (RAG) chatbot within the published book. This chatbot, utilizing the OpenAI Agents SDK with Google Gemini, FastAPI, Neon Serverless Postgres database, and Qdrant Cloud Free Tier, must be able to answer user questions about the book's content, including answering questions based only on text selected by the user.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask General Questions About Book Content (Priority: P1)

A student is reading Module 2: Sensors & Perception and wants to understand the difference between lidar and depth cameras. They open the chat interface, type "What's the difference between lidar and depth cameras?", and receive an answer with references to specific lessons.

**Why this priority**: This is the core value proposition of the RAG chatbot - answering questions about book content using semantic search. Without this, the feature has no value.

**Independent Test**: Can be fully tested by asking 10 questions across all 4 modules and verifying responses cite correct module/lesson names and contain accurate information from the book.

**Acceptance Scenarios**:

1. **Given** a student is on any lesson page, **When** they open the chat interface and type "What is ROS2?", **Then** the chatbot responds within 3 seconds with an answer sourced from Module 1 content and displays "Source: Module 1: ROS2 Nervous System - Lesson 1"

2. **Given** a student asks a question about NVIDIA Isaac, **When** the chatbot searches the embedded content, **Then** it retrieves relevant chunks from Module 3 with similarity score >0.7 and includes them in the response

3. **Given** a student asks "How do I implement navigation in ROS2?", **When** the chatbot processes the query, **Then** it returns information from multiple relevant lessons (e.g., Module 1: Lesson 3 and Module 2: Lesson 4) with proper source attribution

4. **Given** a student types a multi-sentence question with 200+ words, **When** the chatbot receives it, **Then** it processes the query successfully and responds with relevant content

5. **Given** a student asks a follow-up question "Can you explain that in simpler terms?", **When** the chatbot has context of the last 5 messages, **Then** it refers to the previous answer and provides a simplified explanation

---

### User Story 2 - Ask Questions About Selected Text (Priority: P1)

A student is reading about URDF robot descriptions and selects a complex paragraph about joint types. A tooltip appears with "Ask about this" button. They click it and type "What does this mean?", receiving an answer specifically about the selected text with surrounding context.

**Why this priority**: This is the unique differentiator - contextual Q&A based on user-selected text. This is explicitly required in the Phase 2 goal and cannot be deprioritized.

**Independent Test**: Can be fully tested by selecting 5 different text snippets across different modules, asking questions about them, and verifying responses prioritize the selected text context.

**Acceptance Scenarios**:

1. **Given** a student selects at least 10 characters of text on a lesson page, **When** they release the mouse, **Then** a tooltip appears near the selection with "Ask about this" button within 200ms

2. **Given** a student has selected text "ROS2 uses DDS for inter-process communication", **When** they click "Ask about this" and type "What is DDS?", **Then** the chatbot receives both the selected text AND the surrounding paragraphs as context

3. **Given** a student selects a code block example, **When** they ask "What does this code do?", **Then** the chatbot's response prioritizes explaining the selected code over general search results

4. **Given** a student selects text from Module 2 Lesson 3, **When** they ask a question about it, **Then** the chatbot includes metadata (module, lesson, section) from the source lesson in the retrieval context

5. **Given** a student selects less than 10 characters, **When** they release the mouse, **Then** no tooltip appears (selection too short)

6. **Given** a student selects text and asks a question, then asks a follow-up without selecting new text, **When** the follow-up is processed, **Then** the original selected text context is still available in conversation history

---

### User Story 3 - Multi-Turn Conversation with Context (Priority: P2)

A student asks "What is ROS2?", then follows up with "How does it differ from ROS1?" without re-typing context. The chatbot understands the follow-up refers to ROS2 from the previous message and provides a comparative answer.

**Why this priority**: Multi-turn conversations create a natural learning dialogue. While important, the chatbot can provide value with single-turn Q&A first (P1 stories).

**Independent Test**: Can be fully tested by conducting 5 multi-turn conversations (3-5 messages each) and verifying the chatbot maintains context across turns.

**Acceptance Scenarios**:

1. **Given** a student has asked 3 previous questions in the chat, **When** they ask a 4th question with pronouns like "it" or "that", **Then** the chatbot resolves pronouns using the last 5 messages as context

2. **Given** a student asks "What are the main sensors?" then "Which one is most accurate?", **When** the second question is processed, **Then** the chatbot understands "which one" refers to sensors from the previous message

3. **Given** a student has 6 messages in conversation history, **When** they ask a new question, **Then** only the last 5 messages are included in the agent's context (to manage token limits)

4. **Given** a student closes and reopens the chat interface, **When** they return within 7 days, **Then** their conversation history is restored from browser localStorage

5. **Given** a student's conversation history is older than 7 days, **When** they open the chat interface, **Then** the old history is cleared and a fresh conversation starts

---

### User Story 4 - Cross-Lesson Knowledge Retrieval (Priority: P2)

A student asks "How do I use sensors with Isaac Sim?", which requires knowledge from both Module 2 (Sensors) and Module 3 (Isaac). The chatbot retrieves relevant chunks from both modules and synthesizes an answer.

**Why this priority**: Demonstrates the power of semantic search across the entire book. Important for comprehensive answers but not required for MVP.

**Independent Test**: Can be fully tested by asking 5 questions that require information from multiple modules and verifying responses include sources from 2+ modules.

**Acceptance Scenarios**:

1. **Given** a student asks a question that spans multiple modules, **When** the chatbot searches Qdrant, **Then** it retrieves top-k chunks (k=10) regardless of module boundaries

2. **Given** a student asks "How do VLAs use ROS2?", **When** the chatbot retrieves content, **Then** it includes chunks from Module 1 (ROS2) and Module 4 (VLA) with proper source attribution for each

3. **Given** a student asks about a concept mentioned in multiple lessons, **When** the chatbot responds, **Then** it synthesizes information from all relevant lessons and cites them (e.g., "Source: Module 1 Lesson 2, Module 3 Lesson 1")

4. **Given** a student is currently on Module 2 Lesson 3 page, **When** they ask a question answered in Module 4, **Then** the chatbot still retrieves and provides the Module 4 content (not restricted by current page)

---

### User Story 5 - Receive Helpful Error Messages and Suggestions (Priority: P3)

A student asks "How do I install ROS2 on Windows?", which is outside the book's scope (book focuses on concepts, not installation tutorials). The chatbot responds: "I couldn't find information about ROS2 installation in the book. However, you might find these lessons helpful: Module 1: ROS2 Fundamentals."

**Why this priority**: Improves user experience when queries fail, but the chatbot must work for in-scope queries first (P1-P2).

**Independent Test**: Can be fully tested by asking 10 out-of-scope questions and verifying the chatbot gracefully declines and suggests related lessons.

**Acceptance Scenarios**:

1. **Given** a student asks a question with no semantic matches above 0.7 similarity, **When** the chatbot processes the query, **Then** it responds: "I couldn't find specific information about that in the book. Could you rephrase your question or try asking about: [suggested topics from metadata]"

2. **Given** a student asks about a topic completely outside the book (e.g., "What's the weather today?"), **When** the chatbot evaluates the query, **Then** it responds: "That question is outside the scope of this Physical AI and Humanoid Robotics book. I can help you with questions about ROS2, sensors, NVIDIA Isaac, or vision-language-action models."

3. **Given** a student asks a vague question like "Tell me about robots", **When** the chatbot processes it, **Then** it suggests: "That's a broad topic! Here are some areas covered in the book: ROS2 Nervous System, Sensors & Perception, AI Brain with Isaac, Vision-Language-Action. What would you like to learn about?"

4. **Given** a student's query results in a Qdrant error (service down), **When** the backend catches the exception, **Then** it returns HTTP 503 with message: "The chatbot is temporarily unavailable. Please try again in a moment."

5. **Given** a student's query results in a Gemini API error (rate limit), **When** the backend catches the exception, **Then** it returns HTTP 429 with message: "Too many requests. Please wait a moment and try again."

6. **Given** a student asks a question that matches content from incomplete lessons (e.g., quizzes without answers), **When** the chatbot retrieves those chunks, **Then** it filters them out and only returns substantive lesson content

---

### User Story 6 - Chat Interface Visual Feedback (Priority: P3)

A student types a question and clicks "Send". While the chatbot is processing (1-3 seconds), they see a typing indicator with animated dots. When the response arrives, it fades in smoothly with source citations at the bottom.

**Why this priority**: Enhances user experience with visual polish, but the chatbot must work functionally first.

**Independent Test**: Can be fully tested by sending 5 queries and observing loading states, animations, and response rendering.

**Acceptance Scenarios**:

1. **Given** a student submits a question, **When** the request is sent to the backend, **Then** the send button is disabled and a "typing..." indicator appears within 100ms

2. **Given** the chatbot is generating a response, **When** the response streams back (if SSE is implemented), **Then** the text appears word-by-word in real-time

3. **Given** the chatbot response includes source citations, **When** the response is rendered, **Then** source citations appear in a distinct style (e.g., gray text, smaller font, at the bottom)

4. **Given** a student scrolls through chat history, **When** they reach earlier messages, **Then** all messages remain visible and properly formatted

5. **Given** the Docusaurus theme is switched to dark mode, **When** the chat interface is visible, **Then** it adapts to dark mode colors automatically

---

### User Story 7 - Anonymous User with Browser Storage (Priority: P3)

A student uses the chatbot without creating an account. Their conversation history is stored in browser localStorage. They close the browser, return 2 days later, and their previous conversation is still there.

**Why this priority**: Anonymous usage is required (per constitution), but persistence is a nice-to-have for MVP.

**Independent Test**: Can be fully tested by having a conversation, closing the browser, reopening, and verifying history persists.

**Acceptance Scenarios**:

1. **Given** an anonymous student has a conversation with 5 messages, **When** they close the browser tab, **Then** the conversation is saved to localStorage with a timestamp

2. **Given** an anonymous student returns after 3 days, **When** they open the chat interface, **Then** their previous conversation (within 7-day window) is loaded and displayed

3. **Given** an anonymous student's localStorage is full, **When** the chatbot tries to save a new message, **Then** it removes the oldest conversations first (FIFO) and saves the new message

4. **Given** an anonymous student clears their browser data, **When** they open the chat interface, **Then** the conversation history is empty and a new session starts

5. **Given** an anonymous student opens the book in two browser tabs, **When** they chat in both tabs, **Then** each tab maintains its own independent conversation (no cross-tab sync for anonymous users)

---

### Edge Cases

**Content Retrieval:**
- What happens when a student asks a question using terminology not in the book (e.g., "AGI" instead of "AI")?
  - The chatbot should still attempt semantic search and may find related content (e.g., AI, artificial intelligence) if similarity >0.7

- What happens when Qdrant returns 0 results (similarity <0.7 for all chunks)?
  - The chatbot invokes error handling (User Story 5) and suggests related topics

- What happens when a student selects text from a non-lesson page (e.g., homepage, quiz page)?
  - Text selection handler still works, but the context may not have lesson metadata (graceful degradation)

**Multi-turn Conversations:**
- What happens when a student asks 20 questions in one session (exceeds 5-message context window)?
  - Only the last 5 messages are sent to the agent; older messages remain in browser history but not in LLM context

- What happens when conversation history exceeds localStorage quota (typically 5-10MB)?
  - Oldest conversations are deleted (FIFO) until new conversation fits

**API Errors:**
- What happens when Gemini API returns a 500 error?
  - Backend catches exception, logs error, returns HTTP 503 to frontend with user-friendly message

- What happens when Neon Postgres is unreachable?
  - For anonymous users (Phase 2), Postgres is only used for future features, so chatbot continues working with browser-only storage

- What happens when Qdrant Cloud is down?
  - Backend catches exception, returns HTTP 503: "Search is temporarily unavailable"

**Input Validation:**
- What happens when a student submits an empty query?
  - Frontend validates and shows error: "Please enter a question"

- What happens when a student submits a 10,000-character query?
  - Backend validates max length (e.g., 2000 chars), truncates with warning, or rejects with 400 error

- What happens when a student tries to inject HTML/JavaScript in the query?
  - Backend sanitizes all inputs; frontend escapes all rendered content to prevent XSS

**Rate Limiting:**
- What happens when a student sends 50 queries in 1 minute?
  - After 100 requests/hour (per constitution), backend returns HTTP 429 with retry-after header

**Browser Compatibility:**
- What happens when a student uses an old browser without localStorage?
  - Chatbot works but conversation history is lost on page refresh (graceful degradation)

- What happens when a student disables JavaScript?
  - Chat interface does not render; fallback message: "Please enable JavaScript to use the chatbot"

---

## Requirements *(mandatory)*

### Functional Requirements

**Core Chat Functionality:**
- **FR-001**: System MUST accept text queries from users via a chat interface embedded in Docusaurus lesson pages
- **FR-002**: System MUST generate responses using Google Gemini (gemini-2.5-flash) via OpenAI Agents SDK within 3 seconds (95th percentile)
- **FR-003**: System MUST retrieve semantically relevant content from Qdrant Cloud using Cohere embeddings (embed-english-v3.0)
- **FR-004**: System MUST include source citations in responses (format: "Source: Module X: Module Name - Lesson Y: Lesson Name")
- **FR-005**: System MUST support queries up to 2000 characters in length

**Text Selection Feature:**
- **FR-006**: System MUST detect when a user selects text on a lesson page (minimum 10 characters)
- **FR-007**: System MUST display a tooltip near the selected text with an "Ask about this" action button
- **FR-008**: System MUST include the selected text AND surrounding paragraphs as context when processing selected-text queries
- **FR-009**: System MUST prioritize selected-text context over general semantic search results when both are available

**Multi-turn Conversations:**
- **FR-010**: System MUST maintain conversation context of the last 5 user-assistant message pairs
- **FR-011**: System MUST include conversation history in the agent's context when processing follow-up questions
- **FR-012**: System MUST store conversation history in browser localStorage for anonymous users
- **FR-013**: System MUST expire conversation history older than 7 days from localStorage

**Content Search & Retrieval:**
- **FR-014**: System MUST search across all 4 modules (16 lessons) without restricting by current page
- **FR-015**: System MUST retrieve top-10 content chunks from Qdrant based on semantic similarity
- **FR-016**: System MUST filter chunks with similarity score <0.7 (threshold for relevance)
- **FR-017**: System MUST extract and include metadata (module, lesson, section, tags) with each retrieved chunk

**Error Handling & Edge Cases:**
- **FR-018**: System MUST suggest alternative questions or related lessons when no content matches the query (similarity <0.7 for all chunks)
- **FR-019**: System MUST recognize out-of-scope queries (e.g., installation instructions, non-robotics topics) and politely decline with suggestions
- **FR-020**: System MUST return user-friendly error messages when external APIs (Gemini, Qdrant, Neon) fail
- **FR-021**: System MUST sanitize all user inputs to prevent SQL injection, XSS, and other security vulnerabilities
- **FR-022**: System MUST validate query length and reject queries exceeding 2000 characters with HTTP 400

**Rate Limiting & Performance:**
- **FR-023**: System MUST enforce rate limit of 100 requests per hour per user (tracked by session ID for anonymous users)
- **FR-024**: System MUST respond to rate-limited requests with HTTP 429 and "Retry-After" header
- **FR-025**: System MUST complete Qdrant searches within 500ms (95th percentile)
- **FR-026**: System MUST complete Gemini LLM calls within 2 seconds (95th percentile)

**Frontend Integration:**
- **FR-027**: System MUST render chat interface as a fixed bottom-right floating widget on all lesson pages
- **FR-028**: System MUST support expand/collapse interaction for the chat widget
- **FR-029**: System MUST display loading indicators (typing animation) while processing queries
- **FR-030**: System MUST adapt to Docusaurus light/dark theme automatically
- **FR-031**: System MUST be accessible via keyboard navigation (Tab, Enter, Escape)
- **FR-032**: System MUST persist chat open/closed state in browser sessionStorage

**Backend API:**
- **FR-033**: System MUST expose POST `/api/chat/query` endpoint accepting `{ query: str, selected_text?: str, lesson_id?: str, conversation_history?: array }`
- **FR-034**: System MUST return responses in format `{ response: str, sources: array<{module: str, lesson: str}>, conversation_id: str, timestamp: str }`
- **FR-035**: System MUST expose GET `/api/health` endpoint for deployment verification
- **FR-036**: System MUST enforce CORS policy restricting origins to `https://anusbutt.github.io` and `http://localhost:3000`

**Deployment & Environment:**
- **FR-037**: Backend MUST be deployed to Railway with auto-deploy from GitHub repository
- **FR-038**: System MUST use environment variables for all secrets (GEMINI_API_KEY, COHERE_API_KEY, QDRANT_API_KEY, NEON_DATABASE_URL)
- **FR-039**: System MUST log all queries, responses, and errors to Railway logs for monitoring
- **FR-040**: System MUST expose metrics endpoint (optional) for monitoring response times and error rates

---

### Key Entities

**Conversation**:
- Represents a multi-turn dialogue between a user and the chatbot
- Attributes: conversation_id (UUID), messages (array of Message), created_at (timestamp), expires_at (timestamp)
- Storage: Browser localStorage for anonymous users
- Lifecycle: Created on first query, expires after 7 days

**Message**:
- Represents a single user query or assistant response in a conversation
- Attributes: role (user|assistant), content (string), timestamp (ISO 8601), sources (array of Source, assistant only), selected_text (string, optional)
- Relationships: Belongs to one Conversation

**Source**:
- Represents a citation to a specific module/lesson in the book
- Attributes: module (string, e.g., "Module 1: ROS2 Nervous System"), lesson (string, e.g., "Lesson 2: Nodes and Topics"), section (string, optional)
- Relationships: Attached to assistant Messages

**ContentChunk**:
- Represents a semantic section of book content stored in Qdrant
- Attributes: chunk_id (UUID), content (string, 500-1000 tokens), embedding (vector[1024]), metadata (module, lesson, section, tags, skills, bloom_level)
- Storage: Qdrant Cloud Free Tier collection `humanoid_robotics_book`
- Lifecycle: Created once during embedding pipeline, static thereafter

**SelectedTextContext**:
- Represents the context around user-selected text
- Attributes: selected_text (string), lesson_id (string), surrounding_paragraphs (string, before and after), metadata (module, lesson, section)
- Lifecycle: Created when user selects text, included in query payload

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

**Functional Success:**
- **SC-001**: Students can ask 10 representative questions about book content and receive accurate responses with correct source citations in 100% of cases
- **SC-002**: Students can select text from any lesson, ask a question about it, and receive a contextual answer in 100% of cases
- **SC-003**: Students can conduct a 5-turn conversation where the chatbot correctly references prior messages in 100% of follow-up questions
- **SC-004**: System retrieves relevant content from Qdrant with similarity >0.7 for 90% of in-scope questions

**Performance Success:**
- **SC-005**: 95% of queries receive responses within 3 seconds (P95 latency)
- **SC-006**: Qdrant searches complete within 500ms for 95% of queries
- **SC-007**: System handles 50 concurrent users without degradation (measured via load testing)

**User Experience Success:**
- **SC-008**: 90% of students successfully complete their first interaction (ask question → receive answer) without errors
- **SC-009**: Chat interface loads and becomes interactive within 1 second of page load
- **SC-010**: Text selection tooltip appears within 200ms of user releasing mouse after selecting ≥10 characters

**Error Handling Success:**
- **SC-011**: 100% of out-of-scope queries receive helpful error messages with suggestions (not generic "error" messages)
- **SC-012**: 100% of API failures (Gemini, Qdrant, Neon down) result in user-friendly error messages, not stack traces
- **SC-013**: System gracefully handles and recovers from 100% of tested edge cases (empty query, oversized query, special characters, etc.)

**Security & Reliability Success:**
- **SC-014**: 0 XSS vulnerabilities in production (verified via security audit)
- **SC-015**: 0 unhandled exceptions reach the user (all errors caught and logged)
- **SC-016**: Rate limiting successfully blocks users after 100 requests/hour in 100% of test cases

**Content Accuracy Success:**
- **SC-017**: 95% of chatbot responses contain information verifiable in the source lessons (manual review of 100 random queries)
- **SC-018**: 0% of chatbot responses contain hallucinated information contradicting book content (manual review)
- **SC-019**: 100% of source citations correctly map to the module/lesson where the information appears

**Deployment Success:**
- **SC-020**: Backend deploys to Railway successfully from GitHub main branch with 0 manual interventions
- **SC-021**: Frontend deploys to GitHub Pages with embedded chat interface visible on all lesson pages
- **SC-022**: All environment variables (GEMINI_API_KEY, COHERE_API_KEY, QDRANT_API_KEY) are configured in Railway without being committed to git

---

## Out of Scope (Explicitly Excluded from Phase 2)

**Authentication & User Accounts:**
- User registration and login (better-auth integration deferred to future)
- Server-side user profiles and preferences
- Cross-device conversation sync
- Email notifications or user dashboards

**Advanced Chatbot Features:**
- Voice input/output (speech-to-text, text-to-speech)
- Image uploads for visual question-answering
- Code execution or interactive coding environment
- Personalized learning paths based on user progress

**Content Management:**
- Admin interface for editing book content
- Real-time content updates without re-embedding
- Version control for chatbot responses
- A/B testing different response formats

**Analytics & Monitoring:**
- User analytics dashboard (query patterns, popular topics)
- Chatbot performance dashboard (beyond basic logs)
- User feedback collection system (thumbs up/down on responses)
- Conversation analytics (average length, topics discussed)

**Advanced RAG Features:**
- Hybrid search (combining keyword + semantic search)
- Re-ranking retrieved chunks with cross-encoder models
- Query expansion or rephrasing
- Multi-modal embeddings (text + images + code)

**Internationalization:**
- Multi-language support (chatbot only supports English)
- Localized content or translations

**Scalability Beyond Free Tier:**
- Auto-scaling infrastructure
- Production-grade database (Neon free tier sufficient for Phase 2)
- CDN for chatbot assets
- Advanced caching strategies (Redis, etc.)

---

## Technical Constraints

**API Rate Limits:**
- Google Gemini Free Tier: 15 requests per minute, 1500 requests per day
- Cohere Free Tier: 100 API calls per minute, 10,000 per month
- Qdrant Cloud Free Tier: 1GB storage, unlimited queries
- Neon Postgres Free Tier: 0.5GB storage (not heavily used in Phase 2)

**Browser Storage Limits:**
- localStorage: ~5-10MB per domain (varies by browser)
- Conversation history must fit within this limit or implement FIFO deletion

**Deployment Constraints:**
- Railway Free Tier: $5/month usage credit, limited CPU/memory
- GitHub Pages: Static hosting only (no server-side rendering for frontend)

**Security Requirements:**
- All API keys MUST be stored as Railway environment variables
- CORS MUST restrict origins to production domain + localhost
- All user inputs MUST be sanitized before processing
- No PII storage for anonymous users (only conversation text)

---

## Dependencies

**External Services:**
- Google Gemini API (LLM provider)
- Cohere API (embeddings provider)
- Qdrant Cloud (vector database)
- Neon Serverless Postgres (user data, minimal usage in Phase 2)
- Railway (backend hosting)
- GitHub Pages (frontend hosting)

**Internal Dependencies:**
- Phase 1 book content (all 16 lessons) must be complete and published
- Docusaurus site must be live at https://anusbutt.github.io/hackathon-phase-01/

**Development Tools:**
- OpenAI Agents SDK (agents library)
- FastAPI (backend framework)
- React + TypeScript (frontend components)
- Pytest (backend testing)
- Playwright (E2E testing)

---

## Open Questions

**None** - All major decisions have been documented in the constitution and confirmed:
- LLM provider: Gemini 2.5 Flash ✅
- Embeddings: Cohere embed-english-v3.0 ✅
- Deployment: Railway ✅
- Authentication: Anonymous-only ✅
- Storage: Browser localStorage (7 days) ✅
- Multi-turn: Last 5 messages ✅
- Source format: Module/lesson names only ✅
- MVP scope: All 4 modules ✅
- Error handling: Suggest alternatives ✅

---

## Risk Analysis

**High-Risk Items:**
1. **Gemini API Rate Limits**: 15 requests/min may be insufficient for concurrent users
   - Mitigation: Implement request queuing and user-facing rate limit messages

2. **Cohere Free Tier Exhaustion**: 10,000 calls/month could be consumed during embedding generation
   - Mitigation: Generate embeddings once, cache in Qdrant; use Cohere only for new content

3. **Qdrant Search Quality**: Similarity threshold of 0.7 may be too strict or too lenient
   - Mitigation: Test with 100 sample queries and adjust threshold (0.65-0.75 range)

4. **Browser localStorage Overflow**: Conversation history may exceed 5MB limit for active users
   - Mitigation: Implement FIFO deletion and warn users when approaching limit

**Medium-Risk Items:**
1. **Railway Free Tier Exhaustion**: $5/month credit may run out with heavy usage
   - Mitigation: Monitor usage closely; implement request caching

2. **Text Selection UX on Mobile**: Tooltip positioning and touch interactions may be challenging
   - Mitigation: Test on mobile devices early; provide fallback (copy-paste into chat)

3. **CORS Issues in Production**: Misconfigured CORS could block frontend requests
   - Mitigation: Test CORS thoroughly in staging environment before production deploy

**Low-Risk Items:**
1. **Docusaurus Theme Compatibility**: Chat interface may not perfectly match theme
   - Mitigation: Use Docusaurus CSS variables for consistent theming

2. **Conversation History Bugs**: Edge cases in localStorage management
   - Mitigation: Comprehensive unit tests for conversation history CRUD operations

---

## Acceptance Checklist

Before marking this feature as complete, verify:

- [ ] All P1 user stories have passing acceptance scenarios (User Story 1, 2)
- [ ] All P2 user stories have passing acceptance scenarios (User Story 3, 4)
- [ ] All P3 user stories have passing acceptance scenarios (User Story 5, 6, 7)
- [ ] All 40 functional requirements (FR-001 to FR-040) are implemented
- [ ] All 22 success criteria (SC-001 to SC-022) are met with measurement evidence
- [ ] All edge cases listed have documented handling behavior
- [ ] Backend deployed to Railway and accessible via public endpoint
- [ ] Frontend deployed to GitHub Pages with visible chat interface
- [ ] All environment variables configured in Railway (not committed to git)
- [ ] CORS policy restricts to production domain + localhost
- [ ] Rate limiting enforced (100 req/hour per user)
- [ ] Error handling tested for all external API failures
- [ ] Input validation prevents XSS, SQL injection, oversized queries
- [ ] Text selection feature works on 5 different lesson pages
- [ ] Multi-turn conversation tested with 5-turn dialogue
- [ ] Conversation history persists across browser sessions (within 7 days)
- [ ] Qdrant collection populated with embeddings for all 16 lessons
- [ ] Manual review confirms 95% response accuracy on 100 sample queries
- [ ] Load testing confirms system handles 50 concurrent users
- [ ] Security audit confirms 0 XSS vulnerabilities
- [ ] Documentation updated (README, deployment guide, API docs)

---

## Constitution Compliance Check

This specification aligns with the following constitutional principles:

✅ **Phase 2 Goal**: Addresses all requirements (RAG chatbot, OpenAI Agents SDK, Gemini LLM, Cohere embeddings, Qdrant Cloud, Neon Postgres, text selection feature)

✅ **Integrated RAG Architecture**: Uses specified tech stack (Gemini, Cohere, FastAPI, Qdrant, Neon, Railway)

✅ **Tool-Based Data Retrieval**: Spec defines 5 custom tools (search_book_content, get_selected_text_context, get_lesson_metadata, get_user_profile, save_conversation)

✅ **Pure RAG Implementation**: Qdrant Cloud Free Tier for vectors, Cohere for embeddings, no OpenAI vector storage

✅ **Frontend-Backend Integration**: React components (ChatInterface.tsx, TextSelectionHandler.tsx), API endpoints defined

✅ **User Experience Focus**: Anonymous users, browser storage, 7-day retention, multi-turn conversations

✅ **Quality Standards**: Performance targets (<3s response, <500ms search), security (sanitization, CORS), testing (unit, integration, E2E)

✅ **Deployment Strategy**: Railway backend, GitHub Pages frontend, environment variables for secrets

✅ **Spec-Driven Development**: This spec written BEFORE plan/tasks/implementation; no vibe-coding permitted
