<!--
Sync Impact Report:
- Version change: 1.0.0 → 2.0.0 → 2.0.1 → 2.1.0
- Ratified: 2025-12-28
- Last Amendment: 2025-12-30
- Rationale: Enhanced Phase 2 specifications with comprehensive RAG chatbot architecture; specified Railway as backend platform; changed to Gemini LLM and Cohere embeddings
- Changes in this amendment:
  * Added Phase 2 Goal statement with explicit requirements
  * Enhanced Phase 2 Core Principles with detailed tech stack specifications
  * Added 5 required custom tools with @function_tool implementation patterns
  * Expanded Pure RAG Implementation with Qdrant Cloud Free Tier details
  * Added concrete embedding pipeline strategy (text-embedding-3-small, chunking strategy)
  * Enhanced Frontend-Backend Integration with TypeScript examples and API endpoints
  * Added Neon Postgres database schema (users, conversations, user_progress tables)
  * Added detailed data flow architecture diagram
  * Created 7-stage implementation process with feature branch naming
  * Enhanced Quality Standards with performance targets, security requirements, testing strategy
  * Added repository structure for Phase 2 with backend directory layout
  * Added environment management (.env.example, .gitignore updates)
  * Added deployment checklist and rollback strategy
  * Added Phase 2 Compliance Requirements and Code Review Checklist
  * (v2.0.1) Specified Railway as the definitive backend deployment platform
  * (v2.1.0) Changed LLM provider from OpenAI to Google Gemini (gemini-2.5-flash)
  * (v2.1.0) Changed embeddings provider from OpenAI to Cohere (embed-english-v3.0)
  * (v2.1.0) Updated environment variables (GEMINI_API_KEY, COHERE_API_KEY)
  * (v2.1.0) Updated embedding vector dimensions (1536 → 1024 for Cohere)
- Templates requiring updates:
  ✅ plan-template.md - Constitution Check section aligns with principles
  ✅ spec-template.md - User stories support book chapter planning and RAG chatbot
  ✅ tasks-template.md - Task structure supports chapter pipeline and Phase 2 stages
  ⚠️ Consider creating RAG-specific templates for Phase 2 (tool contracts, API endpoints)
- Follow-up TODOs:
  * Create spec for 005-rag-chatbot following enhanced Phase 2 guidelines
  * Set up Qdrant Cloud Free Tier account
  * Set up Neon Serverless Postgres database
  * Create backend directory structure
-->

# Physical AI & Humanoid Robotics Book Constitution (v1.0 - Book Writing Phase)

## Core Principles

### I. Content-First, RAG-Ready Structure (NON-NEGOTIABLE)

Every chapter MUST be written with future RAG (Retrieval-Augmented Generation) integration in mind:

- **Clear semantic chunking**: Use consistent heading hierarchy (H2 for sections, H3 for subsections)
- **Rich frontmatter metadata**: Every lesson includes `title`, `skills`, `learning_objectives`, `cognitive_load`, `tags`
- **Modular content blocks**: Sections can be shown/hidden independently for personalization
- **Summary pairs**: Each main lesson file MUST have a corresponding `.summary.md` file
- **Self-contained explanations**: No dangling references; each section explains concepts fully

**Rationale**: Phase 2 will add chatbot and personalization. Content structure determines RAG quality. Refactoring content later is expensive and error-prone.

### II. Spec-Driven Chapter Development

No content is written without a specification:

- **User defines learning goals** → Engineer drafts chapter spec → User approves
- Each chapter spec MUST include:
  - Learning objectives (measurable)
  - Target audience assumptions (CS students with Python knowledge)
  - Key concepts to cover
  - Code example requirements (conceptual snippets, not full tutorials)
  - Success criteria (what student should be able to do after)
- **Chapter pipeline executes only after spec approval**

**Rationale**: Prevents scope creep, ensures consistency, enables measurable outcomes.

### III. Subagent Pipeline Architecture

Content creation follows a **9-step agent pipeline** for every chapter:

**Pipeline Flow**:
```
Spec (User Approved)
  ↓
[1] Outline Agent → structured subsections
  ↓
[2] Chapter Content Agent → full MDX content
  ↓
[3] Case Study Generator → real-world robotics examples
  ↓
[4] Code Example Agent → ROS2/Python/URDF snippets
  ↓
[5] Technical Reviewer Agent → validates accuracy (ROS2, Gazebo, URDF concepts)
  ↓
[6] Structure & Style Agent → enforces heading hierarchy, frontmatter, summaries
  ↓
[7] Frontmatter Agent → generates compliant YAML metadata
  ↓
[8] Docusaurus Integration Agent → converts to final MDX
  ↓
[9] Review Checkpoint → User approval before commit
```

**Agent Contracts** (MANDATORY):
- **Input schema**: What the agent needs (e.g., topic, learning goals, outline)
- **Output schema**: What it produces (e.g., structured markdown, JSON metadata)
- **Acceptance criteria**: How to validate success
- **Failure modes**: What can go wrong, retry strategy

**Implementation**: Claude Code Subagents (using Task tool with custom agent prompts)

**Stateless Requirement**: Agents have no memory between invocations; all context passed via inputs.

**Composability**: Agents can be reused in Phase 2 for chatbot response validation, content updates, personalization.

**Rationale**: Systematic quality, parallel execution, reusable for Phase 2, demonstrates multi-agent reasoning (hackathon bonus points).

### IV. Book Structure Standards

**Repository Organization** (Monorepo):
```
book-source/
├── docs/
│   └── 13-Physical-AI-Humanoid-Robotics/
│       ├── README.md                          (Chapter overview)
│       ├── 01-ros2-nervous-system/            (Module 1 - 4 lessons)
│       │   ├── README.md                      (Module overview)
│       │   ├── 01-ros2-fundamentals.md
│       │   ├── 01-ros2-fundamentals.summary.md
│       │   ├── 02-nodes-topics-services.md
│       │   ├── 02-nodes-topics-services.summary.md
│       │   ├── 03-python-rclpy-bridge.md
│       │   ├── 03-python-rclpy-bridge.summary.md
│       │   ├── 04-urdf-humanoid-basics.md
│       │   ├── 04-urdf-humanoid-basics.summary.md
│       │   ├── 05-capstone-project.md
│       │   ├── 05-capstone-project.summary.md
│       │   └── 06-quiz.md
│       ├── 02-digital-twin-simulation/         (Module 2 - 4 lessons)
│       ├── 03-nvidia-isaac-ai-brain/          (Module 3 - 4 lessons)
│       └── 04-vision-language-action/         (Module 4 - 4 lessons)
├── src/                                        (Docusaurus React components)
├── static/                                     (Images, assets)
├── docusaurus.config.ts
├── sidebars.ts
└── package.json
```

**Naming Conventions**:
- Chapter: `##-Topic-In-Title-Case` (e.g., `13-Physical-AI-Humanoid-Robotics`)
- Modules: `##-module-name-kebab-case` (e.g., `01-ros2-nervous-system`)
- Lessons: `##-lesson-topic.md` + `##-lesson-topic.summary.md`
- Capstone always numbered `05-capstone-project.md`
- Quiz always numbered `06-quiz.md`

**Content Scope**:
- **4 modules** (ROS2, Simulation, Isaac, VLA)
- **4 lessons per module** (main content + capstone + quiz = 6 files per lesson)
- **16 total lessons** (4 × 4)
- **Audience**: CS students with Python knowledge
- **Code depth**: Conceptual snippets (not full working tutorials)

### V. Quality Gate: Technical Accuracy (NON-NEGOTIABLE)

All technical content MUST pass Technical Reviewer Agent before commit:

**Review Scope**:
- ROS2 concepts (nodes, topics, services, parameters)
- URDF syntax and semantics
- Gazebo physics simulation accuracy
- Python/rclpy code correctness
- NVIDIA Isaac Sim workflows
- Navigation (Nav2) patterns

**Failure Response**: Agent retries with corrections; human review only if agent cannot resolve.

**Rationale**: Technical errors in educational content erode trust. Automated review catches errors before user sees them.

### VI. Incremental Commits & Feature Branches

**Git Workflow**:
- **Feature branch per chapter**: `feature/module-##-lesson-##`
- **Commit after each pipeline step**: e.g., "Add outline for ROS2 fundamentals", "Add technical review for URDF lesson"
- **Commit message format**: `docs(module-##): <imperative verb> <what changed>`
  - Examples:
    - `docs(module-01): add outline for ROS2 fundamentals`
    - `docs(module-02): add code examples for Gazebo physics`
    - `docs(module-01): fix technical review feedback on URDF syntax`
- **Merge to main**: After user approves final lesson (all 9 pipeline steps complete)
- **No direct commits to main**

**Rationale**: Traceable progress, rollback safety, clear history for debugging.

### VII. Minimal & Iterative Agent Development

**Agents are NOT created upfront. They are built when**:
- A task repeats ≥3 times
- A format becomes standardized
- A workflow becomes obvious

**First Iteration**: Manual work to establish pattern
**Second Iteration**: Template-based
**Third Iteration**: Build agent

**Avoids**: Premature abstraction, over-engineering, unused agents

**Rationale**: YAGNI principle. Focus on delivering content first, optimize second.

## Book Structure Standards

### Frontmatter Schema (MANDATORY)

Every lesson file MUST include:

```yaml
---
title: "Lesson Title"
sidebar_position: ##
skills:
  - name: "ROS2 Fundamentals"
    proficiency_level: "beginner"
    category: "robotics-middleware"
    bloom_level: "understand"
    digcomp_area: "technical-concepts"
    measurable_at_this_level: "explain pub/sub model"
learning_objectives:
  - objective: "Understand ROS2 nodes and topics"
    proficiency_level: "beginner"
    bloom_level: "understand"
    assessment_method: "quiz + capstone project"
cognitive_load:
  new_concepts: 5
  assessment: "moderate - builds on Python knowledge"
differentiation:
  extension_for_advanced: "Explore ROS2 services and actions"
  remedial_for_struggling: "Review Python basics first"
tags: ["ros2", "robotics", "middleware"]
generated_by: "agent"
created: "YYYY-MM-DD"
last_modified: "YYYY-MM-DD"
---
```

### Content Structure Pattern

```markdown
# Main Title

## What Is [Concept]?
[Clear definition, context]

## Why [Concept] Matters for Physical AI
[Real-world relevance, motivation]

### Key Principles
[Core ideas, enumerated]

### 💬 AI Colearning Prompt
> Prompt students to explore with Claude/ChatGPT

### 🎓 Expert Insight
> Advanced perspective or common pitfall

## Practical Example
[Code snippet with explanation]

### 🤝 Practice Exercise
> Hands-on task for students

## Summary
[Key takeaways, 3-5 bullets]

## Next Steps
[What comes next in the module]
```

### Code Example Standards

- **Language**: Python 3.11+ (type hints mandatory)
- **Length**: 10-30 lines (conceptual, not production)
- **Format**: Fenced code blocks with syntax highlighting
- **Explanation**: Line-by-line or block-level comments
- **Execution**: Not required to run (conceptual snippets)

Example:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    """Publishes messages to a ROS2 topic."""

    def __init__(self) -> None:
        super().__init__('simple_publisher')
        self.publisher = self.create_publisher(String, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)

    def publish_status(self) -> None:
        msg = String()
        msg.data = 'Robot operational'
        self.publisher.publish(msg)
```

## Subagent Architecture

### Agent Taxonomy

**A) Authoring Agents**:
1. **Outline Agent**: Takes topic → outputs structured subsections
2. **Chapter Content Agent**: Takes outline → writes full MDX content
3. **Case Study Generator**: Creates real-world robotics examples
4. **Code Example Agent**: Generates Python/ROS2/URDF snippets

**B) Validation Agents**:
5. **Technical Reviewer Agent**: Validates ROS2/URDF/Gazebo accuracy
6. **Structure & Style Agent**: Enforces heading hierarchy, summaries, frontmatter

**C) Integration Agents**:
7. **Frontmatter Agent**: Generates compliant YAML metadata
8. **Docusaurus Integration Agent**: Converts to final MDX with sidebar config

### Agent Contract Template

Every agent MUST define:

```markdown
## Agent Name: [Name]

### Input Schema
- `topic`: string (required) - lesson topic
- `learning_goals`: array<string> (required) - what students should learn
- `context`: object (optional) - prior knowledge assumptions

### Output Schema
- `type`: "markdown" | "json" | "yaml"
- `structure`: [describe output format]

### Acceptance Criteria
- [ ] Output follows template structure
- [ ] No placeholder text (e.g., [TODO], [FILL])
- [ ] Meets length requirements (if applicable)
- [ ] Passes validation (syntax, schema)

### Failure Modes
- **Invalid topic**: Retry with clarification prompt
- **Missing context**: Request user input
- **Format violation**: Auto-fix with Structure & Style Agent
```

### Agent Reusability (Phase 2)

Same agents will be used for:
- **Chatbot answer validation**: Technical Reviewer validates RAG responses
- **Personalization**: Structure & Style Agent adapts content based on user profile
- **Content updates**: Chapter Content Agent regenerates sections with new information
- **Knowledge extraction**: Frontmatter Agent generates embeddings metadata

## Development Workflow

### Chapter Development Process

**Phase 0: Specification**
1. User defines learning goals for chapter
2. Engineer drafts chapter spec (learning objectives, key concepts, success criteria)
3. User reviews and approves spec

**Phase 1: Pipeline Execution**
1. Create feature branch: `feature/module-##-lesson-##`
2. Execute 9-step agent pipeline (Outline → Content → Case Study → Code → Review → Style → Frontmatter → Docusaurus → User Review)
3. Commit after each step
4. User approves final output

**Phase 2: Integration**
1. Merge feature branch to main
2. Deploy to GitHub Pages (auto via GitHub Actions)
3. Validate live deployment

**Phase 3: Iteration**
1. Collect feedback (if any)
2. Create new feature branch for fixes
3. Re-run affected pipeline steps
4. Merge and redeploy

### Review Checkpoints

**After Each Pipeline Step**:
- Agent completes task
- Output validated against acceptance criteria
- Commit to feature branch
- Proceed to next step

**Before Merge**:
- User reviews complete lesson (all files: main, summary, capstone, quiz)
- Validates against chapter spec
- Approves or requests changes

**After Deployment**:
- User validates live page on GitHub Pages
- Checks navigation, formatting, code blocks, images

## Quality Standards

### Priority Ranking (1 = Highest)

1. **Code Quality** (tests, maintainability, documentation)
2. **Performance** (fast page loads, efficient builds)
3. **Security** (no secrets in repo, safe dependencies)

### Code Quality Gates

**Documentation**:
- Every agent MUST have contract documentation
- Every module MUST have README with overview
- Every lesson MUST have summary file

**Maintainability**:
- DRY principle: Reuse agents, don't duplicate logic
- Clear naming: Descriptive file/folder names
- Consistent structure: Follow book structure standards

**Testing** (Phase 1 - Manual):
- User validates content accuracy
- Technical Reviewer Agent validates code/concepts
- No automated tests for content (yet)

### Performance Targets

**Build Time**: <2 minutes for full Docusaurus build
**Page Load**: <3 seconds for lesson page
**Image Optimization**: <200KB per image, WebP format preferred

### Security Requirements

**Secrets**: Never commit API keys, tokens, credentials
**Dependencies**: Use `npm audit` to check for vulnerabilities
**Access Control**: GitHub Pages is public (no sensitive content)

## Governance

### Constitution Authority

This constitution supersedes all other practices for both Phase 1 (Book Writing) and Phase 2 (RAG Chatbot Development).

**Scope**: Book content creation, agent architecture, development workflow, RAG chatbot, personalization, authentication
**Out of Scope**: Advanced AI features beyond specified scope, third-party integrations not mentioned

### Amendment Process

**Version Bump Rules**:
- **MAJOR (x.0.0)**: Backward-incompatible changes (e.g., remove pipeline step, change structure)
- **MINOR (1.x.0)**: New principles, new agents, expanded guidance
- **PATCH (1.0.x)**: Clarifications, wording fixes, typo corrections

**Amendment Steps**:
1. Propose change with rationale
2. User approval required
3. Update constitution file
4. Propagate to dependent templates (plan, spec, tasks)
5. Document in Sync Impact Report (HTML comment at top of file)
6. Create PHR for amendment

### Compliance Review

**Every PR MUST**:
- Follow git workflow (feature branch, commit messages)
- Pass Technical Reviewer Agent
- Include summary file for main lesson
- Have compliant frontmatter

**Agent Pipeline MUST**:
- Execute all 9 steps in order
- Validate against agent contracts
- Commit after each step

**Content MUST**:
- Match approved chapter spec
- Target CS students with Python knowledge
- Use conceptual code snippets (not full tutorials)
- Be RAG-ready (clear chunking, rich metadata)

## Physical AI & Humanoid Robotics Book Constitution (v2.0 - RAG Chatbot Phase)

### Phase 2 Goal

Build and embed a Retrieval-Augmented Generation (RAG) chatbot within the published book. This chatbot must:

1. **Answer questions about the book's content** using OpenAI Agents/ChatKit SDK with custom tools
2. **Answer questions based on user-selected text** from any lesson page
3. **Leverage Phase 1 RAG-ready content** through Qdrant Cloud Free Tier vector database
4. **Provide seamless integration** within the existing Docusaurus educational platform

### Phase 2 Core Principles

#### I. Integrated RAG Architecture (NON-NEGOTIABLE)

The RAG chatbot MUST be built with the following technology stack:

**Required Technologies**:
- **AI Agent Framework**: OpenAI Agents SDK for conversational intelligence and tool orchestration
- **LLM Provider**: Google Gemini (`gemini-2.5-flash`) via OpenAI-compatible API
- **Embeddings Provider**: Cohere (free tier) for semantic content embeddings
- **Backend Service**: FastAPI (Python 3.11+) for API endpoints and business logic
- **Vector Database**: Qdrant Cloud Free Tier (1GB cluster) for semantic content retrieval
- **User Database**: Neon Serverless Postgres for user profiles, conversations, and preferences
- **Frontend Integration**: Docusaurus React components for embedded chat interface
- **Custom Tools**: Python functions with `@function_tool` decorator for data access

**Rationale**: Provides scalable, maintainable architecture that leverages the RAG-ready content from Phase 1 while enabling contextual responses based on user-selected text. Gemini 2.5 Flash offers fast, cost-effective responses. Cohere free tier provides high-quality embeddings at no cost. Qdrant Cloud Free Tier provides sufficient capacity for all Phase 1 content (~16 lessons).

#### II. Tool-Based Data Retrieval (OpenAI Agents SDK Pattern)

All data retrieval MUST be handled through custom Python functions decorated with `@function_tool` from the OpenAI Agents SDK:

**Required Custom Tools**:

1. **`search_book_content`** - Semantic search in Qdrant
   - Input: user query string, optional filters (module, lesson, tags)
   - Output: list of relevant content chunks with metadata
   - Purpose: Retrieve context-relevant book content for RAG responses

2. **`get_selected_text_context`** - Process user-selected text
   - Input: selected text, source lesson ID, surrounding context
   - Output: enriched context with lesson metadata and related sections
   - Purpose: Answer questions specifically about user-highlighted content

3. **`get_lesson_metadata`** - Fetch lesson frontmatter
   - Input: lesson ID or module+lesson number
   - Output: learning objectives, skills, tags, cognitive load
   - Purpose: Provide educational context for personalized responses

4. **`get_user_profile`** - Retrieve user preferences from Neon Postgres
   - Input: user ID
   - Output: learning history, preferences, current module progress
   - Purpose: Enable personalized responses based on user journey

5. **`save_conversation`** - Store chat history in Neon Postgres
   - Input: user ID, query, response, timestamp, lesson context
   - Output: conversation record ID
   - Purpose: Maintain conversation history for follow-up questions

**Tool Implementation Pattern**:
```python
from openai.agents import function_tool

@function_tool
def search_book_content(query: str, module_filter: str | None = None) -> list[dict]:
    """
    Search the book content using semantic similarity in Qdrant.

    Args:
        query: The user's question or search query
        module_filter: Optional module name to restrict search scope

    Returns:
        List of relevant content chunks with metadata and similarity scores
    """
    # Implementation: query Qdrant, return top-k results
    pass
```

**LLM Initialization Pattern**:
```python
from agents import Agent, Runner, OpenAIChatCompletionsModel, AsyncOpenAI
import os
from dotenv import load_dotenv

load_dotenv()

# Initialize Gemini via OpenAI-compatible API
external_client = AsyncOpenAI(
    api_key=os.getenv("GEMINI_API_KEY"),
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)

# Configure the LLM model
llm_model = OpenAIChatCompletionsModel(
    model="gemini-2.5-flash",
    openai_client=external_client
)

# Create agent with custom tools
agent = Agent(
    name="RAG_Assistant",
    model=llm_model,
    tools=[search_book_content, get_selected_text_context, get_user_profile, save_conversation]
)
```

**Rationale**: OpenAI Agents SDK automatically handles tool discovery, parameter validation, and execution flow. This pattern maintains clear separation of concerns while enabling the agent to dynamically access both book content and user context. Gemini provides cost-effective, fast responses via OpenAI-compatible API.

#### III. Pure RAG Implementation (Qdrant Cloud Free Tier)

The system MUST implement a pure RAG system using Qdrant Cloud Free Tier without relying on OpenAI's built-in vector storage:

**Content Embedding Pipeline**:
1. **Extract Phase 1 Content**: Parse all MDX lesson files from `book-source/docs/13-Physical-AI-Humanoid-Robotics/`
2. **Chunk Strategy**:
   - Split by semantic sections (H2/H3 headings)
   - Chunk size: 500-1000 tokens (optimal for retrieval)
   - Maintain frontmatter metadata with each chunk
3. **Generate Embeddings**: Use Cohere `embed-english-v3.0` model (free tier)
4. **Store in Qdrant**: Upload to Qdrant Cloud Free Tier (1GB = ~1M embeddings)
   - Collection name: `humanoid_robotics_book`
   - Vector size: 1024 dimensions (embed-english-v3.0)
   - Metadata fields: module, lesson, section, tags, skills, bloom_level

**Retrieval Flow**:
1. **User Query** → Convert to embedding using same model
2. **Semantic Search** → Query Qdrant with similarity threshold (>0.7)
3. **Context Assembly**:
   - Retrieve top-k chunks (k=5-10)
   - Include user-selected text if provided
   - Add relevant lesson metadata
4. **Response Generation** → Pass assembled context to OpenAI Agent

**User-Selected Text Integration**:
- **Frontend**: JavaScript captures selected text + lesson ID + surrounding paragraphs
- **Backend**: Enriches selected text with related book sections from Qdrant
- **Agent Behavior**: Prioritizes selected text context over general search results

**Qdrant Cloud Free Tier Specifications**:
- Storage: 1GB vector storage
- Capacity: Sufficient for ~16 lessons × ~10 sections × ~2 chunks = ~320 embeddings
- API Access: REST API + Python client (`qdrant-client`)
- Cost: Free (no credit card required)

**Rationale**: Provides full control over the retrieval process, ensures compatibility with existing RAG-ready content from Phase 1, and leverages free tier resources efficiently.

#### IV. Frontend-Backend Integration

The chatbot interface MUST be seamlessly integrated into the existing Docusaurus site:

**Chat Interface Component** (`src/components/ChatInterface.tsx`):
- **Placement**: Fixed bottom-right corner with expandable/collapsible UI
- **Design**: Matches Docusaurus theme (light/dark mode support)
- **State Management**: React hooks for chat history and user input
- **API Communication**: Fetch API to FastAPI backend endpoints

**Text Selection Capture** (`src/components/TextSelectionHandler.tsx`):
```typescript
// JavaScript pattern for capturing selected text
document.addEventListener('mouseup', () => {
  const selectedText = window.getSelection()?.toString();
  if (selectedText && selectedText.length > 10) {
    // Show "Ask about this" button near selection
    // Capture: selectedText, lessonId, surroundingContext
    // Send to chatbot with context
  }
});
```

**API Endpoints** (FastAPI Backend):
- **POST `/api/chat/query`**: Main chat endpoint
  - Input: `{ query: str, selected_text?: str, lesson_id?: str, user_id?: str }`
  - Output: `{ response: str, sources: list[str], conversation_id: str }`
- **POST `/api/chat/feedback`**: User feedback on responses
  - Input: `{ conversation_id: str, rating: int, comment?: str }`
- **GET `/api/user/history`**: Retrieve conversation history
  - Input: `user_id` (query param)
  - Output: `{ conversations: list[dict] }`

**Real-time Communication**:
- **Primary**: REST API with streaming responses (SSE - Server-Sent Events)
- **Fallback**: Standard REST API with polling
- **Session**: JWT tokens for user authentication (optional)

**User Experience Features**:
1. **Context Awareness**: Chat knows current lesson from URL
2. **Quick Actions**: "Explain this concept", "Show code example", "Related lessons"
3. **Visual Feedback**: Loading states, typing indicators, source citations
4. **Accessibility**: Keyboard navigation, screen reader support (ARIA labels)

**Rationale**: Provides a natural learning experience without disrupting the existing educational flow. Text selection feature enables targeted question-answering about specific content.

#### V. User Experience Focus

The chatbot MUST prioritize educational value and user experience:

- **Contextual Responses**: Answers based on both book content and user-selected text
- **Educational Tone**: Responses aligned with educational objectives from Phase 1
- **Progressive Disclosure**: Complex topics explained in digestible steps
- **Error Handling**: Graceful degradation when content is not found

**Rationale**: Ensures the chatbot enhances rather than replaces the learning experience provided by the Phase 1 content.

### Phase 2 Architecture Standards

#### Tech Stack Requirements

**Frontend**:
- Docusaurus v3.9.2 (TypeScript) - maintaining compatibility with Phase 1
- React 19.0.0 with TypeScript for type-safe components
- Custom hooks for chat state management
- JavaScript/TypeScript for text selection capture

**Backend**:
- FastAPI (latest stable, >=0.100.0) - for API endpoints
- Python 3.11+ - consistent with Phase 1 code examples
- OpenAI Agents SDK - for conversational agent orchestration
- `openai` Python package - for AsyncOpenAI client (Gemini API compatibility)
- `cohere` Python package - for embeddings generation
- `qdrant-client` - for Qdrant Cloud interactions
- `psycopg2` or `asyncpg` - for Neon Postgres connections
- Pydantic v2 - for request/response validation

**Database & Storage**:
- **Qdrant Cloud Free Tier**: Vector storage for book content embeddings
- **Neon Serverless Postgres**: Relational data for users and conversations

#### Neon Postgres Database Schema

**Required Tables**:

```sql
-- User profiles and preferences
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    username VARCHAR(255) UNIQUE,
    email VARCHAR(255) UNIQUE,
    created_at TIMESTAMP DEFAULT NOW(),
    current_module VARCHAR(100),
    current_lesson VARCHAR(100),
    preferences JSONB DEFAULT '{}'::jsonb
);

-- Conversation history
CREATE TABLE conversations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id),
    query TEXT NOT NULL,
    response TEXT NOT NULL,
    selected_text TEXT,
    lesson_id VARCHAR(255),
    sources JSONB,
    created_at TIMESTAMP DEFAULT NOW(),
    rating INTEGER CHECK (rating >= 1 AND rating <= 5)
);

-- User learning progress
CREATE TABLE user_progress (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id),
    module VARCHAR(100),
    lesson VARCHAR(100),
    completed BOOLEAN DEFAULT FALSE,
    time_spent_seconds INTEGER DEFAULT 0,
    quiz_score INTEGER,
    last_accessed TIMESTAMP DEFAULT NOW(),
    UNIQUE(user_id, module, lesson)
);

-- Indexes for performance
CREATE INDEX idx_conversations_user_id ON conversations(user_id);
CREATE INDEX idx_conversations_created_at ON conversations(created_at DESC);
CREATE INDEX idx_user_progress_user_id ON user_progress(user_id);
```

#### Data Flow Architecture

```
User Interaction (lesson page with chat interface)
  ↓
[Frontend] Text selection captured (optional)
  ↓
[Frontend] Query + context sent to FastAPI
  ↓
[FastAPI] POST /api/chat/query endpoint
  ↓
[OpenAI Agent] Initialized with custom tools
  ↓
┌─────────────────────────────────────────┐
│ Agent decides which tools to call:      │
│ 1. search_book_content (Qdrant)         │
│ 2. get_selected_text_context (Qdrant)  │
│ 3. get_user_profile (Neon Postgres)    │
│ 4. get_lesson_metadata (Static/Cache)   │
└─────────────────────────────────────────┘
  ↓
[OpenAI Agent] Generates response with retrieved context
  ↓
[FastAPI] save_conversation tool stores to Neon Postgres
  ↓
[Frontend] Response displayed with source citations
```

#### Tool Contract Standards

Every custom tool MUST define:

```markdown
## Tool Name: [Name]

### Function Signature
- `function_name(params)`: description of what the function does

### Input Schema
- `param1`: type (required/optional) - description
- `param2`: type (required/optional) - description

### Output Schema
- `type`: return type
- `description`: what is returned

### Purpose
- Clear description of what this tool accomplishes
```

#### Integration Points

**Docusaurus Integration**:
- New React components in `book-source/src/components/`
- Updated `docusaurus.config.ts` to include chatbot functionality
- Modified lesson templates to support text selection

**API Endpoints**:
- `/api/chat` - Handle chat interactions
- `/api/tools/*` - Custom tool endpoints
- `/api/user/*` - User data management

### Phase 2 Development Workflow

#### Specification Requirements (MANDATORY)

Before any Phase 2 implementation, create a detailed spec following the spec-template.md that includes:

1. **Chatbot Functionality Requirements**:
   - Question-answering capabilities (general vs. selected-text)
   - Conversation context management
   - Multi-turn dialog support
   - Source attribution requirements

2. **Content Scope**:
   - All Phase 1 modules and lessons
   - Frontmatter metadata utilization
   - Code example accessibility
   - Quiz and capstone project content

3. **User-Selected Text Feature**:
   - Minimum selection length (e.g., 10 characters)
   - Context window around selection
   - UI/UX for "Ask about this" interaction
   - Handling of code blocks vs. prose

4. **User Profile & Personalization**:
   - Anonymous vs. authenticated users
   - Progress tracking requirements
   - Conversation history retention
   - Privacy and data handling

#### Implementation Process

**Stage 0: Specification & Planning**
1. User defines Phase 2 requirements
2. Engineer drafts comprehensive spec in `specs/005-rag-chatbot/spec.md`
3. Engineer creates architectural plan in `specs/005-rag-chatbot/plan.md`
4. User approves spec and plan
5. Engineer creates task breakdown in `specs/005-rag-chatbot/tasks.md`

**Stage 1: Infrastructure Setup** (Feature Branch: `feature/rag-chatbot-infrastructure`)
1. **Qdrant Setup**:
   - Create Qdrant Cloud Free Tier account
   - Create collection `humanoid_robotics_book`
   - Configure API key and endpoint
2. **Neon Postgres Setup**:
   - Create Neon Serverless Postgres database
   - Run schema migration (users, conversations, user_progress tables)
   - Configure connection string
3. **FastAPI Backend Scaffold**:
   - Initialize FastAPI project structure
   - Set up environment variables (.env)
   - Create health check endpoint
4. **Environment Configuration**:
   ```env
   OPENAI_API_KEY=sk-...
   QDRANT_URL=https://....qdrant.io
   QDRANT_API_KEY=...
   NEON_DATABASE_URL=postgresql://...
   CORS_ORIGINS=https://anusbutt.github.io
   ```

**Stage 2: Content Embedding Pipeline** (Feature Branch: `feature/rag-content-pipeline`)
1. **Content Extraction Script** (`scripts/extract_lessons.py`):
   - Parse all MDX files
   - Extract frontmatter metadata
   - Split content by H2/H3 sections
2. **Embedding Generation** (`scripts/generate_embeddings.py`):
   - Generate embeddings using `text-embedding-3-small`
   - Batch processing (100 chunks per batch)
   - Store embeddings with metadata in Qdrant
3. **Validation**:
   - Verify all lessons embedded (16 lessons)
   - Test semantic search queries
   - Validate metadata fields

**Stage 3: Custom Tools Implementation** (Feature Branch: `feature/rag-tools`)
1. Implement all 5 required tools with `@function_tool` decorator
2. Unit tests for each tool
3. Integration tests with Qdrant and Neon
4. Documentation for each tool contract

**Stage 4: OpenAI Agent Integration** (Feature Branch: `feature/openai-agent`)
1. Configure OpenAI Agent with custom tools
2. Implement main chat endpoint (`POST /api/chat/query`)
3. Add streaming response support (SSE)
4. Error handling and fallback logic
5. Response validation and safety checks

**Stage 5: Frontend Components** (Feature Branch: `feature/chat-interface`)
1. Create `ChatInterface.tsx` component
2. Create `TextSelectionHandler.tsx` component
3. Integrate into Docusaurus layout
4. Add CSS/styling (theme-aware)
5. Implement loading states and error handling

**Stage 6: Integration & Testing** (Feature Branch: `feature/rag-integration`)
1. Connect frontend to FastAPI backend
2. End-to-end testing of chat flow
3. Test selected-text feature on actual lessons
4. Performance testing (response times)
5. User acceptance testing

**Stage 7: Deployment** (Merge to Main)
1. Deploy FastAPI backend to cloud platform (Render/Railway/Vercel)
2. Update Docusaurus config with backend URL
3. Deploy to GitHub Pages (automated via GitHub Actions)
4. Monitor logs and performance
5. Document deployment process

### Quality Standards for Phase 2

#### Technical Requirements (NON-NEGOTIABLE)

**Performance Targets**:
- **Response Time**: <3 seconds for typical queries (95th percentile)
- **Embedding Search**: <500ms for Qdrant vector search
- **Database Queries**: <200ms for Neon Postgres lookups
- **Frontend Load**: Chat interface adds <50KB to page size

**Reliability**:
- **Availability**: 99% uptime for chatbot service
- **Error Handling**: Graceful degradation when APIs are unavailable
- **Rate Limiting**: Prevent abuse (e.g., 100 requests/hour per user)
- **Retry Logic**: Automatic retries for transient failures

**Security** (CRITICAL):
- **API Keys**: Never commit to repository; use environment variables only
- **CORS**: Restrict origins to production Docusaurus domain
- **Input Validation**: Sanitize all user inputs (SQL injection, XSS prevention)
- **Data Privacy**: No PII stored without explicit consent
- **Authentication**: Optional JWT-based auth for user tracking
- **Secrets Management**: Use Railway environment variables (not committed to repository)

**Scalability**:
- **Concurrent Users**: Handle 50+ concurrent users on free tier resources
- **Database Connections**: Connection pooling for Neon Postgres
- **Caching**: Cache lesson metadata and frequently accessed content
- **Cost Control**: Monitor OpenAI API usage; implement monthly budget alerts

#### Content Accuracy & Educational Quality

**Factual Correctness**:
- **Source Attribution**: Every response MUST cite specific lesson sections
- **Content Boundary**: Responses based ONLY on Phase 1 book content
- **Hallucination Prevention**: Use RAG with strict relevance threshold (>0.7)
- **Code Examples**: Only reference existing code from lessons

**Context Awareness**:
- **Selected Text Priority**: When user selects text, prioritize that context
- **Lesson Context**: Include current lesson metadata in agent context
- **Multi-turn Dialogs**: Maintain conversation history for follow-up questions
- **Scope Awareness**: Recognize when queries are outside book content scope

**Educational Value**:
- **Learning Objectives Alignment**: Responses support lesson learning objectives
- **Pedagogical Tone**: Explanatory, patient, encouraging (not directive)
- **Progressive Complexity**: Adjust explanations based on user's current module
- **Practice Encouragement**: Suggest related exercises and capstone projects

**Quality Assurance**:
1. **Automated Testing**:
   - Unit tests for all custom tools (90% coverage minimum)
   - Integration tests for RAG pipeline
   - End-to-end tests for chat flow
2. **Manual Testing**:
   - Sample 20 representative questions per module
   - Validate responses against ground truth (lesson content)
   - User acceptance testing with CS students
3. **Monitoring**:
   - Track response accuracy (user feedback ratings)
   - Log all queries and responses for review
   - Monitor retrieval quality (relevance scores)

#### Testing Strategy

**Unit Tests** (pytest):
```python
# Example: Test search_book_content tool
def test_search_book_content():
    result = search_book_content("What is ROS2?")
    assert len(result) > 0
    assert result[0]['score'] > 0.7
    assert 'ros2' in result[0]['content'].lower()
```

**Integration Tests**:
- Test Qdrant connection and search
- Test Neon Postgres CRUD operations
- Test OpenAI Agent with mock tools
- Test selected-text context assembly

**End-to-End Tests** (Playwright):
- User selects text on lesson page
- User types question in chat interface
- Response appears with source citations
- User can view conversation history

### Phase 2 Deployment Strategy

The RAG chatbot will be deployed as an extension to the existing GitHub Pages site with separate backend infrastructure:

#### Deployment Architecture

**Frontend Deployment** (GitHub Pages):
- **Source**: Same repository, `main` branch
- **Build**: GitHub Actions workflow (existing)
- **Output**: Static site with embedded React chat components
- **URL**: https://anusbutt.github.io/hackathon-phase-01/
- **Updates**: Automatic on push to main

**Backend Deployment** (Railway):
- **Platform**: Railway (free tier with $5/month credit)
- **Runtime**: Python 3.11+ with FastAPI
- **Environment Variables**: GEMINI_API_KEY, COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL
- **Endpoint**: Public API endpoint (e.g., https://yourapp.up.railway.app)
- **Auto-deploy**: Connected to GitHub repository (feature branch or main)

**Database Services** (Serverless):
1. **Qdrant Cloud Free Tier**:
   - 1GB storage (sufficient for all embeddings)
   - No credit card required
   - REST API access
2. **Neon Serverless Postgres**:
   - Free tier: 0.5GB storage, 1 compute unit
   - Auto-suspend when idle
   - Connection pooling via Neon proxy

#### Repository Structure for Phase 2

**Updated Directory Layout**:
```
hackathon-phase-01/
├── book-source/                    # Phase 1 (Docusaurus site)
│   ├── docs/                       # Educational content
│   ├── src/
│   │   └── components/
│   │       ├── ChatInterface.tsx   # NEW: Chat UI component
│   │       └── TextSelectionHandler.tsx  # NEW: Text selection
│   └── docusaurus.config.ts        # Updated with backend URL
├── backend/                        # NEW: Phase 2 backend
│   ├── app/
│   │   ├── main.py                 # FastAPI app entry point
│   │   ├── agents/
│   │   │   └── chatbot_agent.py    # OpenAI Agent configuration
│   │   ├── tools/
│   │   │   ├── search_content.py   # Custom tools
│   │   │   ├── user_profile.py
│   │   │   └── conversation.py
│   │   ├── models/
│   │   │   └── schemas.py          # Pydantic models
│   │   └── config/
│   │       └── settings.py         # Environment config
│   ├── scripts/
│   │   ├── extract_lessons.py      # Content extraction
│   │   └── generate_embeddings.py  # Embedding generation
│   ├── tests/
│   │   ├── test_tools.py
│   │   └── test_agent.py
│   ├── requirements.txt            # Python dependencies
│   └── Dockerfile                  # Optional: containerization
├── specs/
│   └── 005-rag-chatbot/            # NEW: Phase 2 spec
│       ├── spec.md
│       ├── plan.md
│       └── tasks.md
├── .github/workflows/
│   ├── deploy.yml                  # Updated: includes backend tests
│   └── backend-deploy.yml          # NEW: Backend deployment
└── README.md                        # Updated with Phase 2 info
```

#### Environment Management

**.env.example** (Backend):
```env
# Google Gemini Configuration (LLM Provider)
GEMINI_API_KEY=your-gemini-api-key-here
GEMINI_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/
GEMINI_MODEL=gemini-2.5-flash

# Cohere Configuration (Embeddings Provider)
COHERE_API_KEY=your-cohere-api-key-here
COHERE_EMBEDDING_MODEL=embed-english-v3.0

# Qdrant Cloud Configuration
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key-here
QDRANT_COLLECTION_NAME=humanoid_robotics_book

# Neon Postgres Configuration
NEON_DATABASE_URL=postgresql://user:password@hostname/database

# API Configuration
CORS_ORIGINS=https://anusbutt.github.io,http://localhost:3000
API_RATE_LIMIT=100  # requests per hour per user
```

**.gitignore Additions**:
```
# Phase 2 additions
backend/.env
backend/__pycache__/
backend/.pytest_cache/
backend/venv/
*.pyc
```

#### Deployment Checklist

**Pre-Deployment**:
- [ ] All Phase 2 tests passing (unit + integration)
- [ ] Qdrant collection populated with embeddings
- [ ] Neon Postgres schema migrated
- [ ] Environment variables configured on cloud platform
- [ ] CORS settings restrict to production domain
- [ ] API rate limiting enabled
- [ ] Logging and monitoring configured

**Deployment Steps**:
1. Deploy backend to Railway
2. Verify backend health endpoint: `GET /api/health`
3. Test chat endpoint with sample query: `POST /api/chat/query`
4. Update Docusaurus config with backend URL (Railway endpoint)
5. Deploy frontend via GitHub Actions
6. End-to-end test on production site
7. Monitor logs for errors via Railway dashboard

**Post-Deployment**:
- Monitor OpenAI API usage (set budget alerts)
- Track user feedback ratings
- Review conversation logs weekly
- Optimize slow queries if needed

#### Rollback Strategy

If critical issues arise:
1. Disable chat interface via feature flag in Docusaurus config
2. Display maintenance message to users
3. Investigate and fix issue in feature branch
4. Re-deploy after testing
5. Re-enable chat interface

---

## Governance & Compliance

### Constitution Updates

**Version**: 2.1.0
**Ratified**: 2025-12-28
**Last Amended**: 2025-12-30
**Amendments**:
- v2.0.0 (2025-12-30): Enhanced Phase 2 specifications with detailed RAG chatbot architecture, OpenAI Agents SDK integration, Qdrant Cloud Free Tier configuration, text selection feature implementation, Neon Postgres schema, deployment strategy, and quality standards.
- v2.0.1 (2025-12-30): Specified Railway as the definitive backend deployment platform (previously Render/Railway/Vercel options).
- v2.1.0 (2025-12-30): Changed LLM provider to Google Gemini (gemini-2.5-flash) and embeddings provider to Cohere (embed-english-v3.0) for cost-efficiency. Updated environment variables, tech stack, and embedding pipeline specifications.

### Phase 2 Compliance Requirements

**Every Phase 2 PR MUST**:
- Follow spec-driven development (spec → plan → tasks → implementation)
- Include unit tests for custom tools (90% coverage)
- Pass integration tests (Qdrant + Neon + OpenAI Agent)
- Document all API endpoints and tools
- Update deployment documentation if infrastructure changes
- Include security review (API keys, CORS, input validation)

**Phase 2 Code Review Checklist**:
- [ ] All `@function_tool` decorators properly typed
- [ ] OpenAI Agent initialized with correct tools
- [ ] Qdrant queries use appropriate similarity threshold
- [ ] User inputs sanitized (SQL injection, XSS prevention)
- [ ] Error handling with graceful degradation
- [ ] Response includes source citations
- [ ] Conversation history stored in Neon Postgres
- [ ] Frontend components are theme-aware (light/dark mode)
- [ ] Text selection handler validates minimum length
- [ ] API endpoints have rate limiting

---

**Constitution Authority**: This document supersedes all other development practices for Physical AI & Humanoid Robotics Book (Phase 1 & Phase 2).

**Scope**: Content creation, agent architecture, RAG chatbot, frontend/backend integration, deployment, security, testing.

**Out of Scope**: Advanced AI features beyond specified requirements, third-party integrations not mentioned, production-scale infrastructure.
