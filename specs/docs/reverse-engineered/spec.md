# Physical AI & Humanoid Robotics — Interactive Textbook Specification

**Version**: 1.0 (Reverse Engineered)
**Date**: 2026-06-26
**Source**: Physical-Ai-Humanoid-Robotics-TextBook

---

## 1. Project Identity

| Attribute | Value |
|---|---|
| **Name** | Physical AI & Humanoid Robotics — Interactive Textbook |
| **Type** | Full-stack Docusaurus + FastAPI RAG application |
| **Frontend** | Docusaurus 3.9.2, React 18, TypeScript 5.6 |
| **Backend** | Python 3.13, FastAPI, uv workspace |
| **LLM** | OpenRouter (Qwen 2.5 72B Instruct) via OpenAI Agents SDK OpenAIChatCompletionsModel |
| **Vector DB** | Qdrant Cloud with Cohere embeddings (embed-multilingual-v3.0) |
| **Package Mgmt** | uv workspace, pyproject.toml at repo root + Backend/ |
| **Hosting** | Frontend: Vercel; Backend: Hugging Face Spaces (Docker) |

---

## 2. System Intent

### 2.1 Target Users

Students and researchers studying Physical AI & Humanoid Robotics.

### 2.2 Core Value Proposition

An interactive textbook where every page has an embedded AI chat assistant grounded in the book content. Users can ask questions, get answers with source citations, and maintain conversation context across sessions.

### 2.3 Key Capabilities

1. **Book Content Rendering (FR8)** — Render 4 modules of book content as a Docusaurus documentation site with sidebar navigation. Modules cover ROS 2 Fundamentals, Perception & Navigation, VLA Systems, and Advanced Topics. Content is authored in Markdown (`.md` files) located under `Frontend/docs/`.
2. **RAG Question Answering (FR1/FR2)** — Answer user questions grounded in book content via a Retrieval-Augmented Generation pipeline. Queries are embedded using Cohere, top-3 chunks are retrieved from Qdrant, and the LLM generates an answer constrained to those chunks.
3. **Token Streaming (FR2)** — Stream response tokens token-by-token via Server-Sent Events (SSE) for real-time display in the frontend ChatKit component.
4. **Conversation Memory (FR6)** — Maintain the last 5 conversation turns per UUID session_id in an in-memory dictionary. Sessions expire after 30 minutes of inactivity (TTL=1800s). Background cleanup runs every 5 minutes.
5. **Off-Topic Guardrails (FR5)** — Detect and block off-topic queries via LLM-based classification. Two implementations exist: the Connection service uses OpenAI SDK `@input_guardrail` + a manual check in the streamed path; the Agent service uses its own `check_off_topic()` via ChatCompletions.
6. **Frontend Chat Persistence (FR7)** — Persist chat history in browser localStorage with multi-session support. Sessions auto-delete after 48 hours. Storage keys follow the pattern `chatkit_active_session`, `chatkit_session:{uuid}`, `chatkit_sessions_order`. All localStorage access is wrapped in SSR-safe `try/catch` blocks.
7. **Vector Context Retrieval (FR4)** — Retrieve relevant chunks from Qdrant vector DB via Cohere embeddings with TOP_K=3. Retrieved chunks are truncated to 6000 characters for LLM context.

### 2.4 Architecture Diagram (Logical)

```
User Browser (Docusaurus + ChatKit)
    |
    | POST /chat or /chat/stream (JSON/SSE)
    v
Connection API (FastAPI, port 8000)
    |--- Rate limiter (100 req/min/IP)
    |--- Input sanitization (strip, 10k limit, injection patterns)
    |--- Guardrail (@input_guardrail / manual)
    |--- Session memory (last 5 turns, 30-min TTL)
    |--- Agent Wrapper (OpenRouter via OpenAIChatCompletionsModel, max_tokens=150)
    |       |--- Context retrieval (sys.path hack -> Retrieval/src/)
    |       |--- Qdrant vector search (TOP_K=3)
    |       |--- Cohere embeddings
    |--- SSE streaming (POST /chat/stream only)
    |
    | (Orchestrator starts both services)
    v
Agent API (FastAPI, port 8001)
    |--- Same RAG pipeline (OpenRouter, Qdrant, Cohere, guardrails)
    |--- Standalone /chat + /chat/stream + /health endpoints
    |--- Dead code: error_handling.py (CircuitBreaker, unused in production)

Retrieval Service (sys.path hack, not pip package)
    |--- Qdrant client wrapper
    |--- Cohere embedding client
    |--- Search with TOP_K chunks

Infrastructure:
    Frontend (Vercel)          -> Docusaurus static site
    Backend (HF Spaces Docker) -> FastAPI app (port 7860 -> 8000)
    Qdrant Cloud               -> Vector storage
    OpenRouter                 -> LLM inference
    Cohere                     -> Embeddings API
```

---

## 3. Functional Requirements

### FR1: Chat Endpoint — POST /chat

**Location**: `Backend/Connection/api.py:112`

| Aspect | Detail |
|---|---|
| **Description** | Process a user message and return an AI-generated response grounded in book content |
| **Request Body** | `{"message": string, "selected_text": string | null, "session_id": string | null}` |
| **Response Body** | `{"response": string, "sources": string[], "is_off_topic": boolean}` |
| **Side Effects** | Updates in-memory session dict with last 5 turns; resets session TTL |
| **Error Codes** | 429 (rate limit), 422 (empty message), 413 (message > 10k chars), 400 (injection detected) |
| **Implementation** | Calls `agent_wrapper.get_response()` which embeds query, retrieves from Qdrant, calls OpenRouter, checks off-topic guardrail |

**Retry Logic** (`Backend/Agent/adapter.py:113`):
- 3 attempts with exponential backoff (2^attempt seconds)
- Timeout: 30 seconds per agent call

### FR2: Streaming Chat — POST /chat/stream

**Location**: `Backend/Connection/api.py:194`, `Backend/Agent/main.py:163`

| Aspect | Detail |
|---|---|
| **Description** | Stream response tokens via SSE for real-time frontend display |
| **Request Body** | Same as FR1 |
| **Response** | SSE stream of JSON events, newline-delimited |
| **Stream Events** | `{"token": "..."}` for each token; final `{"done": true, "session_id": "..."}` |
| **Off-Topic Handling** | First event is `{"type": "off_topic", "content": "..."}` |
| **Errors** | SSE stream may emit error events; connection closed on error |

### FR3: Health Check — GET /health

**Location**: `Backend/Connection/api.py:254`, `Backend/Agent/main.py:201`

| Aspect | Detail |
|---|---|
| **Description** | Verify service is alive and operational |
| **Response Body** | `{"status": "healthy", "timestamp": <ISO datetime>}` |
| **Agent Extra** | Also exposes `/health/database` and `/health/status` endpoints |

### FR4: Vector Search

**Location**: `Backend/Retrieval/src/retrieval/` (accessed via sys.path hack in `Connection/context_switcher.py` and `Agent/agent.py`)

| Aspect | Detail |
|---|---|
| **Description** | Retrieve relevant book chunks from Qdrant based on user query |
| **Input** | User query string, TOP_K=3 (configurable via env) |
| **Output** | List of chunks with content, metadata (module/chapter/section), and relevance score |
| **Embedding Model** | Cohere embed-multilingual-v3.0 |
| **Context Limit** | Retrieved chunks truncated to 6000 characters before LLM injection |
| **Import Mechanism** | sys.path hack — `Retrieval/src/` injected into `sys.path`, then `from retrieval import ...` |
| **Config** | COLLECTION_NAME env var (default: `book_embeddings`), QDRANT_URL, QDRANT_API_KEY |

### FR5: Off-Topic Guardrail

**Location**: `Backend/Connection/guardrails.py`, `Backend/Agent/agent.py:260`

| Aspect | Detail |
|---|---|
| **Description** | Classify whether a query is answerable from book content |
| **Input** | User message + book content context |
| **Output** | Boolean (is_off_topic) |
| **Connection Method** | Uses OpenAI Agents SDK `@input_guardrail` decorator + manual check in streamed path |
| **Agent Method** | Uses `check_off_topic()` function via ChatCompletions call to OpenRouter |
| **Fallback Response** | "I can only answer questions about the book content" |
| **Performance** | max_tokens=150 for guardrail classification calls |

### FR6: Session Memory

**Location**: `Backend/Connection/api.py:33-68`

| Aspect | Detail |
|---|---|
| **Description** | Track last 5 conversation turns per UUID session_id |
| **Storage** | In-memory Python dict: `Dict[str, Dict]` mapping session_id to session data |
| **TTL** | 1800 seconds (30 minutes) from last activity |
| **Cleanup** | Background `asyncio` task runs every 5 minutes, removes expired sessions |
| **Turn Limit** | Last 5 turns only (configurable via SESSION_MEMORY_TURNS env) |
| **Persistence** | None — sessions are ephemeral and lost on service restart |

### FR7: Multi-Session Chat History (Frontend)

**Location**: `Frontend/src/components/ChatKit/ChatKit.tsx`

| Aspect | Detail |
|---|---|
| **Description** | Persist chat sessions in browser localStorage with auto-delete |
| **Storage Keys** | `chatkit_active_session` (current session UUID), `chatkit_session:{uuid}` (messages per session), `chatkit_sessions_order` (ordered list of session UUIDs) |
| **Auto-Delete** | Sessions older than 48 hours are removed on page load |
| **Features** | History panel, delete individual session, refresh button, SSR-safe try/catch around all localStorage calls |
| **SSR Constraint** | All browser-API calls (`localStorage`, `window`) wrapped in `try/catch` blocks — never in module scope |
| **Backend URLs** | Hardcoded in `ChatKit.tsx:14-18` as `BACKEND_URLS` array (localhost:8000 + HF Spaces URL) |

### FR8: Book Content Display

**Location**: `Frontend/docs/`

| Aspect | Detail |
|---|---|
| **Description** | Render book content as a Docusaurus documentation site with sidebar navigation |
| **Content Format** | Markdown (`.md` files) |
| **Module Structure** | 4 modules: Module 1 (ROS 2 Fundamentals), Module 2 (Perception & Navigation), Module 3 (VLA Systems), Module 4 (Advanced Topics) |
| **Chapters** | 8 chapters total across 4 modules |
| **Navigation** | Docusaurus sidebar auto-generated from file structure |

### FR9: Rate Limiting

**Location**: `Backend/Connection/api.py`

| Aspect | Detail |
|---|---|
| **Description** | Prevent API abuse with per-IP rate limiting |
| **Limit** | 100 requests per 60 seconds per IP address |
| **Storage** | In-memory dict (not distributed/Redis-backed) |
| **Response on Limit** | HTTP 429 with rate limit message |
| **Logging** | Logs when rate limit exceeded |

### FR10: Input Sanitization

**Location**: `Backend/Connection/api.py`

| Aspect | Detail |
|---|---|
| **Description** | Sanitize all user inputs before processing |
| **Steps** | 1. `strip()` whitespace 2. Check length <= 10,000 chars 3. Scan for injection patterns (`<script`, `javascript:`, `onerror`, `alert()`) |
| **Response on Violation** | HTTP 400 with descriptive error message |
| **Logging** | Logs injection attempts |

---

## 4. Non-Functional Requirements

### 4.1 Performance

| Metric | Target | Implementation |
|---|---|---|
| **Response Tokens** | 80-100 words per answer | max_tokens=150 for all LLM calls |
| **Context Size** | ~6000 chars of retrieved content | TOP_K=3 chunks, truncated |
| **Retry Backoff** | 2^attempt seconds | 3 retries max |
| **Agent Timeout** | 30 seconds | Configurable per call |
| **LLM Provider** | OpenRouter via OpenAIChatCompletionsModel | Not OpenAI Responses API |
| **Endpoint Latency** | < 10s for typical queries | LLM-dependent (OpenRouter round-trip) |

### 4.2 Security

| Measure | Implementation |
|---|---|
| **Input Sanitization** | `strip()`, length check (10k chars), injection pattern detection (`<script`, `javascript:`, `onerror`, `alert()`) |
| **CORS** | `allow_origins=["*"]` — permissive, should be restricted in production |
| **Rate Limiting** | 100 requests per 60 seconds per IP (in-memory) |
| **Authentication** | None — publicly accessible API |
| **Secrets Management** | `.env` file (gitignored) with live API keys |
| **API Key Exposure** | None in logs or responses |

### 4.3 Reliability

| Measure | Implementation |
|---|---|
| **Retry Decorator** | Exponential backoff with 3 attempts (`Backend/Agent/adapter.py:113`) |
| **Circuit Breaker** | Present in `Backend/Agent/error_handling.py` but **dead code** — only imported by tests |
| **Graceful Degradation** | "I can only answer questions about the book content" fallback for off-topic or retrieval failure |
| **Session Cleanup** | Background task every 5 minutes removes expired in-memory sessions |
| **Empty Qdrant Results** | LLM responds without context rather than breaking |

### 4.4 Observability

| Measure | Implementation |
|---|---|
| **Logging** | Python `logging.basicConfig` at INFO level |
| **Log Events** | Rate limit exceeded, injection attempts, stream errors, Qdrant errors |
| **Health Endpoints** | `/health` on Connection; `/health`, `/health/database`, `/health/status` on Agent |
| **Format** | Plain text logs (not structured JSON) |

### 4.5 Scalability

| Aspect | Current State |
|---|---|
| **Sessions** | In-memory dict — single-process only, not horizontally scalable |
| **Session Storage** | No database — sessions lost on restart |
| **Chat History** | Frontend localStorage — no server-side storage |
| **Rate Limiting** | In-memory — per-process, not shared across instances |
| **C10k Readiness** | Not designed for high concurrency |

---

## 5. System Constraints

### 5.1 External Dependencies

| Dependency | Purpose | Auth Required | Config Env |
|---|---|---|---|
| **OpenRouter API** | LLM provider for chat completions | `OPENROUTER_API_KEY` | `OPENROUTER_BASE_URL`, `OPENROUTER_MODEL` |
| **Qdrant Cloud** | Vector database for chunk storage & retrieval | `QDRANT_API_KEY` | `QDRANT_URL`, `COLLECTION_NAME` |
| **Cohere API** | Text embeddings for query & chunk vectorization | `COHERE_API_KEY` | — |

### 5.2 Data Formats

| Data | Format | Schema |
|---|---|---|
| **API Request/Response** | JSON | As defined in FR1-FR3 |
| **SSE Streaming** | JSON-per-line, newline-delimited | `{"token": "..."}\n`, `{"done": true, "session_id": "..."}\n` |
| **Book Content** | Markdown | `.md` files in `Frontend/docs/` |
| **Conversation Memory** | Python list | `List[Dict[str, str]]` with keys `"user"` and `"assistant"` |
| **Vector Embeddings** | float[] (dim=1024 for Cohere multilingual v3) | Stored in Qdrant points |
| **Environment Config** | `.env` file | Key=value pairs at repo root |

### 5.3 Deployment Context

| Environment | Frontend | Backend | Ports |
|---|---|---|---|
| **Production** | Vercel (static build) | Hugging Face Spaces (Docker) | 7860 (Spaces) |
| **Development** | `npm start` (Docusaurus dev) | `uv run python main.py` or `uv run uvicorn Connection.api:app` | 3000 / 8000, 8001 |
| **Hot-Reload** | Docusaurus live reload | `DEBUG=True` env for uvicorn reload | 3000 / 8000 |

**Build Commands**:
- Frontend: `cd Frontend && npm install && npm run build` (output: `Frontend/build/`)
- Backend Docker: `Backend/Dockerfile`, entry point `python app.py`

### 5.4 Compliance

| Aspect | Status |
|---|---|
| **Data Storage** | No persistent server-side user data storage |
| **Authentication** | None required — public API |
| **GDPR/CCPA** | No controls implemented |
| **Session Data** | Ephemeral in-memory only, lost on restart |
| **Chat History** | Client-side only (localStorage) |

---

## 6. API Contracts

### 6.1 POST /chat

```
POST /chat
Content-Type: application/json

Request:
{
  "message": string,          // required, max 10k chars after strip()
  "selected_text": string | null,  // optional, overrides vector retrieval
  "session_id": string | null      // optional, UUID v4; new session if null
}

Response 200:
{
  "response": string,         // AI-generated answer grounded in book content
  "sources": string[],        // list of source citations (module/chapter references)
  "is_off_topic": boolean     // true if query was classified as off-topic
}

Response 400 (validation):
{
  "detail": "Message cannot be empty" | "Message too long" | "Invalid input detected"
}

Response 422 (malformed):
{
  "detail": "Validation error details from Pydantic"
}

Response 429 (rate limit):
{
  "detail": "Rate limit exceeded. Try again later."
}
```

### 6.2 POST /chat/stream

```
POST /chat/stream
Content-Type: application/json
Accept: text/event-stream

Request:
{
  "message": string,
  "selected_text": string | null,
  "session_id": string | null
}

Response (SSE stream):
event: message
data: {"token": "The"}

event: message
data: {"token": " answer"}

event: message
data: {"token": " is"}

...

event: message
data: {"done": true, "session_id": "uuid-v4-here"}

--- For off-topic queries, first event is:
event: message
data: {"type": "off_topic", "content": "I can only answer questions about the book content."}

--- On error:
event: error
data: {"detail": "Error description"}
```

### 6.3 GET /health

```
GET /health

Response 200:
{
  "status": "healthy",
  "timestamp": "2026-06-26T12:00:00.000000"
}
```

### 6.4 Error Taxonomy

| Status Code | Condition | When |
|---|---|---|
| **200** | Success | All successful requests |
| **400** | Bad Request | Empty message, injection detected, malformed input |
| **413** | Payload Too Large | Message > 10,000 characters |
| **422** | Unprocessable Entity | Invalid JSON, missing required fields |
| **429** | Too Many Requests | >100 requests/min from same IP |
| **500** | Internal Server Error | Qdrant/OpenRouter failures after retries exhausted |

### 6.5 Error Response Format

All errors (except SSE stream errors) return JSON with a `detail` key:
```json
{
  "detail": "Human-readable error message"
}
```

SSE stream errors emit an `error` event type before closing the connection.

---

## 7. Data Model

### 7.1 Qdrant Collection: book_embeddings

| Field | Type | Description |
|---|---|---|
| **id** | UUID (point ID) | Unique chunk identifier |
| **vector** | float[1024] | Cohere embed-multilingual-v3.0 embedding |
| **payload.content** | string | Chunk text content |
| **payload.module** | string | Module name/number |
| **payload.chapter** | string | Chapter name/number |
| **payload.section** | string | Section name/number |
| **payload.version** | string | Book version |

**Collection Config**:
- Distance metric: Cosine
- Created/managed by: `Backend/Ingestion/` pipeline

### 7.2 Session Memory (In-Memory)

```python
# Type: Dict[str, SessionData]
# Key: session_id (UUID v4 string)

SessionData = {
    "history": List[Dict[str, str]],  # [{"user": ..., "assistant": ...}, ...]
    "created_at": float,              # time.time() on creation
    "last_active": float              # time.time() on last request
}

# Constraints:
# - history limited to last 5 turns (configurable)
# - TTL: 1800s from last_active
# - Cleanup: async task every 300s
```

### 7.3 ChatKit localStorage Schema (Frontend)

```typescript
// Key: chatkit_session:{uuid}
// Value: JSON string of:
interface ChatSession {
  id: string;        // UUID v4
  messages: Array<{
    role: "user" | "assistant" | "system";
    content: string;
    timestamp: number;
  }>;
  createdAt: number;
  lastActive: number;
  isOffTopic?: boolean;
}

// Key: chatkit_active_session
// Value: session UUID string

// Key: chatkit_sessions_order
// Value: JSON array of session UUIDs [oldest ... newest]

// Cleanup: sessions with lastActive > 48h are deleted on page load
```

### 7.4 Configuration Model

| Env Variable | Default | Used By | Description |
|---|---|---|---|
| `OPENROUTER_API_KEY` | — | Connection, Agent | OpenRouter API key |
| `OPENROUTER_BASE_URL` | `https://openrouter.ai/api/v1` | Connection, Agent | OpenRouter API base URL |
| `OPENROUTER_MODEL` | `qwen-2.5-72b-instruct` | Connection, Agent | Model identifier |
| `COHERE_API_KEY` | — | Ingestion, Agent, Retrieval | Cohere API key |
| `QDRANT_API_KEY` | — | Ingestion, Retrieval | Qdrant Cloud API key |
| `QDRANT_URL` | — | Ingestion, Retrieval | Qdrant Cloud cluster URL |
| `COLLECTION_NAME` | `book_embeddings` | Ingestion, Retrieval | Qdrant collection name |
| `TOP_K` | `3` | Connection, Agent | Number of chunks to retrieve |
| `SESSION_MEMORY_TURNS` | `5` | Connection | Max conversation turns in session |
| `SESSION_TTL` | `1800` | Connection | Session idle timeout (seconds) |
| `DEBUG` | `False` | All entry points | Enables hot-reload when `True` |

---

## 8. Non-Goals & Out of Scope

The following items are explicitly **excluded** from the current system scope:

| Item | Rationale |
|---|---|
| **User Authentication/Accounts** | Public, open-access textbook |
| **Persistent Server-Side Storage** | Sessions ephemeral; chat history client-only |
| **Multi-User Session Isolation** | No user identity concept |
| **WebSocket Support** | SSE is sufficient for unidirectional streaming |
| **Production-Grade Rate Limiting** | In-memory, per-process, not distributed |
| **API Versioning** | Single version (`/chat`, `/chat/stream`, `/health`) |
| **OpenAPI/Swagger Documentation** | Not auto-generated or published |
| **CI/CD Pipeline** | Not implemented |
| **Code Coverage Tools** | Not configured |
| **Integration/E2E Tests for Connection** | Only Agent service has tests |
| **Structured Logging** | Uses basicConfig plain-text logging |
| **Horizontal Scalability** | Sessions and rate limiting are in-memory |

---

## 9. Known Gaps & Technical Debt

### Gap 1: Dead Production Code — `error_handling.py`

| Attribute | Detail |
|---|---|
| **File** | `Backend/Agent/error_handling.py` |
| **Contents** | 109 lines: CircuitBreaker class + Qdrant error decorator |
| **Problem** | Only imported by `test_core_functionality.py`, never by production code |
| **Impact** | Dead code that could mask real issues if wired in without testing |
| **Recommendation** | Either remove the file or integrate into the production agent/adapter pipeline |

### Gap 2: No Integration Tests for Connection Service

| Attribute | Detail |
|---|---|
| **Files** | `Backend/Connection/` — all production code |
| **Tests** | `pytest` suite exists only under `Backend/Agent/` |
| **Impact** | Changes to the primary user-facing API (Connection) are untested |
| **Recommendation** | Add `pytest` suite for Connection endpoints covering FR1-FR6 |

### Gap 3: sys.path Hack for Retrieval Imports

| Attribute | Detail |
|---|---|
| **Files** | `Backend/Connection/context_switcher.py`, `Backend/Agent/agent.py` |
| **Mechanism** | Inject `Retrieval/src/` into `sys.path`, then `from retrieval import ...` |
| **Impact** | Fragile — breaks if directory structure changes or if Retrieval is packaged as pip install |
| **Recommendation** | Package `Retrieval/` as a proper pip wheel and declare as workspace dependency |

### Gap 4: Hardcoded Backend URLs in Frontend

| Attribute | Detail |
|---|---|
| **File** | `Frontend/src/components/ChatKit/ChatKit.tsx:14-18` |
| **Contents** | `BACKEND_URLS` array hardcodes `localhost:8000` and HF Spaces URL |
| **Impact** | Requires code change to switch environments |
| **Recommendation** | Use environment variable at build time or Docusaurus `customFields` (partially done in `docusaurus.config.ts:31`) |

### Gap 5: 12 Unreferenced Static Images

| Attribute | Detail |
|---|---|
| **Directory** | `Frontend/static/img/` |
| **Contents** | 12 SVG/PNG files with zero source references (including `favicon.ico`, `logo.svg`, 3 `undraw_docusaurus_*` defaults) |
| **Impact** | Increases build output size unnecessarily |
| **Recommendation** | Audit and remove unreferenced assets |

### Gap 6: 3 Unused Gazebo/URDF Simulation Files

| Attribute | Detail |
|---|---|
| **Directory** | `Frontend/assets/` |
| **Contents** | `default_world.sdf`, `humanoid_robot.urdf`, `vla_world.sdf` |
| **Impact** | Confusing to new developers — appears to be simulation content but never referenced |
| **Recommendation** | Move to `Backend/` if simulation integration is planned, or remove |

### Gap 7: No Structured Logging

| Attribute | Detail |
|---|---|
| **Current** | `logging.basicConfig(level=logging.INFO)` — plain text |
| **Impact** | Hard to parse, filter, and query logs in production (HF Spaces) |
| **Recommendation** | Add structured logging (structlog, python-json-logger) |

### Gap 8: CORS `allow_origins=["*"]`

| Attribute | Detail |
|---|---|
| **Location** | Backend FastAPI CORS middleware configuration |
| **Risk** | Any domain can make requests to the API in production |
| **Recommendation** | Restrict to known frontend domains (Vercel, HF Spaces) |

---

## 10. Success Criteria

### 10.1 Functional Success

| # | Criterion | Verification |
|---|---|---|
| SC-F1 | POST /chat returns grounded responses for book-related queries | Manual test with course-typical questions |
| SC-F2 | POST /chat/stream streams tokens incrementally via SSE | Observe token-by-token delivery in browser |
| SC-F3 | Off-topic queries return `is_off_topic=true` with guardrail message | Test with unrelated questions |
| SC-F4 | Session memory maintains last 5 conversation turns | Send 6+ messages in same session, verify context awareness |
| SC-F5 | Frontend ChatKit displays messages, sources, and error states | Visual inspection in Docusaurus |
| SC-F6 | Multi-session history persists across page reloads | Create sessions, reload page, verify history panel |
| SC-F7 | 48h auto-delete removes expired sessions from localStorage | Inspect localStorage keys after setting old timestamps |
| SC-F8 | Rate limiting blocks >100 req/min/IP | 101 rapid requests, observe 429 on last |

### 10.2 Non-Functional Success

| # | Criterion | Target |
|---|---|---|
| SC-NF1 | Response time for typical queries | < 10s (LLM-dependent via OpenRouter) |
| SC-NF2 | Rate limiter enforcement | 100 req/min/IP hard limit |
| SC-NF3 | All health endpoints | Return HTTP 200 |
| SC-NF4 | API key exposure | Zero keys in logs or responses |
| SC-NF5 | SSR build | Succeeds without localStorage errors |
| SC-NF6 | max_tokens compliance | All LLM calls use max_tokens=150 |
| SC-NF7 | Input length enforcement | Messages >10k chars return 413 |
| SC-NF8 | Injection pattern blocking | `<script>`, `javascript:`, `onerror`, `alert()` patterns return 400 |

---

## 11. Acceptance Tests

### Test 1: Book-Content Query

```
Given:  A running Connection API with Qdrant populated with book content
When:   POST /chat with {"message": "What is embodied intelligence?"}
Then:   Response contains answer grounded in book content
        Response.sources is a non-empty array of string citations
        Response.is_off_topic is false
```

### Test 2: Off-Topic Query

```
Given:  A running Connection API
When:   POST /chat with {"message": "What is the weather today?"}
Then:   Response.is_off_topic is true
        Response.response contains "I can only answer questions about the book content"
        Response.sources is an empty array
```

### Test 3: Streaming Chat

```
Given:  A running Connection API
When:   POST /chat/stream with a valid book-related question
Then:   SSE stream delivers multiple {"token": "..."} events
        Final event is {"done": true, "session_id": "<uuid>"}
        (If off-topic) First event is {"type": "off_topic", "content": "..."}
```

### Test 4: Session Memory

```
Given:  A running Connection API
When:   Two messages with same session_id
        First: "What is ROS 2?"
        Second: "What did I just ask about?"
Then:   Second response acknowledges the first question (e.g., "You asked about ROS 2...")
```

### Test 5: Rate Limiting

```
Given:  A running Connection API
When:   101 rapid POST requests to /chat from same IP address
Then:   Requests 1-100 return HTTP 200
        Request 101 returns HTTP 429 with rate limit error message
```

### Test 6: Input Validation

```
Given:  A running Connection API
When:   POST /chat with {"message": ""}
Then:   Response is HTTP 422 with validation error

When:   POST /chat with {"message": "a".repeat(10001)}
Then:   Response is HTTP 413 (or 400) with message-too-long error

When:   POST /chat with {"message": "<script>alert('xss')</script>"}
Then:   Response is HTTP 400 with injection-detected error
```

### Test 7: Health Check

```
Given:  A running Connection API
When:   GET /health
Then:   Response is HTTP 200
        Response.status is "healthy"
        Response.timestamp is a valid ISO datetime string
```

---

## 12. Configuration Reference

### 12.1 env File Template

```bash
# === LLM Provider ===
OPENROUTER_API_KEY=sk-or-v1-...
OPENROUTER_BASE_URL=https://openrouter.ai/api/v1
OPENROUTER_MODEL=qwen-2.5-72b-instruct

# === Embeddings ===
COHERE_API_KEY=...

# === Vector Database ===
QDRANT_API_KEY=...
QDRANT_URL=https://...
COLLECTION_NAME=book_embeddings

# === Runtime ===
TOP_K=3
SESSION_MEMORY_TURNS=5
SESSION_TTL=1800
DEBUG=False
```

### 12.2 File Locations

| File | Purpose |
|---|---|
| `.env` (repo root) | Environment variables loaded by both services |
| `Backend/pyproject.toml` | uv workspace root + project config |
| `Backend/Connection/api.py` | Main FastAPI app (port 8000) |
| `Backend/Agent/main.py` | Standalone Agent FastAPI app (port 8001) |
| `Backend/main.py` | Orchestrator: spawns all 3 backend processes |
| `Backend/app.py` | HF Spaces entry point (port 7860, Connection only) |
| `Backend/Dockerfile` | Container definition for Spaces deployment |
| `Frontend/docusaurus.config.ts` | Docusaurus configuration |
| `Frontend/src/components/ChatKit/ChatKit.tsx` | Main chat UI component |
| `Frontend/vercel.json` | Vercel deployment configuration |

---

## 13. Architecture Decision Records (Retrospective)

### ADR-001: OpenRouter via OpenAIChatCompletionsModel Instead of Responses API

**Context**: LLM provider integration for chat completions.

**Decision**: Use OpenAI Agents SDK `OpenAIChatCompletionsModel` pointed at OpenRouter's `/v1/chat/completions` endpoint, instead of the default Responses API.

**Rationale**: OpenRouter ignores `instructions` and `max_output_tokens` on `/v1/responses`. The ChatCompletions-compatible interface is fully supported.

**Consequences**: The `max_tokens=150` parameter is respected. The SDK's `Runner.run()` works with the model wrapper. Code is slightly more verbose (need to construct the model wrapper explicitly).

### ADR-002: sys.path Hack Over pip Package for Retrieval

**Context**: Importing the Retrieval service from Connection and Agent services.

**Decision**: Inject `Retrieval/src/` into `sys.path` at runtime rather than packaging Retrieval as a pip-installable package.

**Rationale**: Faster development iteration during rapid prototyping. Avoids workspace pip dependency complexity.

**Consequences**: Fragile import mechanism (Gap 3). Import fails if directory structure changes. Must replicate the hack in any new service that needs Retrieval.

### ADR-003: In-Memory Sessions Over Redis/Database

**Context**: Session memory storage for conversation history.

**Decision**: Use a Python in-memory dict with background cleanup task.

**Rationale**: Zero infrastructure dependencies. Sufficient for single-process demo/educational deployment.

**Consequences**: Sessions lost on restart. Not horizontally scalable. Rate limiting is also per-process.

### ADR-004: Client-Side Chat History Over Server-Side Storage

**Context**: Persisting chat history across user sessions.

**Decision**: Store chat history in browser localStorage with 48h auto-delete.

**Rationale**: Compliant with no-user-data-stored requirement. No backend storage costs. SSR-safe with try/catch wrappers.

**Consequences**: History is device-specific. No cross-device sync. 48h limit means older conversations are automatically purged.

### ADR-005: SSE Over WebSockets for Streaming

**Context**: Real-time token delivery to the frontend.

**Decision**: Use Server-Sent Events (SSE) over WebSockets.

**Rationale**: Simpler implementation. Unidirectional streaming is sufficient (client sends via POST, receives via SSE). Native `EventSource` support in browsers. Works with standard HTTP load balancers.

**Consequences**: No bidirectional communication. Reconnection logic must be handled client-side.

---

## 14. Development Quick Reference

### Backend Dev Commands

```powershell
# Start all services (Connection:8000 + Agent:8001 + Retrieval test)
cd Backend; uv run python main.py

# Hot-reload for Connection only
$env:DEBUG="True"; uv run python app.py

# Single service
uv run python -m uvicorn Connection.api:app --port 8000

# Run Agent tests
cd Backend/Agent; pytest
```

### Frontend Dev Commands

```powershell
cd Frontend; npm start

# Typecheck
npm run typecheck

# Build (for Vercel)
npm run build
```

### Architecture Overview

```
Frontend/           (Docusaurus 3.9.2, React 18, TypeScript 5.6)
  ├── src/components/ChatKit/   — SSE streaming chat UI, localStorage multi-session
  ├── docs/                     — Book content (4 modules, 8 chapters, Markdown)
  ├── static/img/               — 15 images (3 referenced, 12 unreferenced)
  └── assets/                   — 3 Gazebo/URDF sim files (unused)

Backend/            (Python 3.13, uv workspace)
  ├── Connection/               — Primary API (port 8000)
  │   ├── api.py                — FastAPI app: /chat, /chat/stream, /health
  │   ├── agent_wrapper.py      — OpenRouter via OpenAIChatCompletionsModel
  │   ├── guardrails.py         — LLM off-topic classification
  │   └── context_switcher.py   — sys.path hack for Retrieval import
  ├── Agent/                    — Standalone RAG (port 8001)
  │   ├── main.py               — /chat, /chat/stream, /health
  │   ├── agent.py              — RAGAgent with process_message
  │   ├── adapter.py            — OpenRouter client with retry
  │   └── error_handling.py     — DEAD CODE (CircuitBreaker, test-only)
  ├── Ingestion/                — Book -> Qdrant pipeline
  ├── Retrieval/                — Vector search (accessed via sys.path)
  ├── main.py                   — Process orchestrator
  └── app.py                    — HF Spaces entry point (port 7860)
```

---

## 15. Revision History

| Date | Version | Author | Changes |
|---|---|---|---|
| 2026-06-26 | 1.0 | Reverse Engineering | Initial specification extracted from codebase analysis |

---

*End of Specification*
