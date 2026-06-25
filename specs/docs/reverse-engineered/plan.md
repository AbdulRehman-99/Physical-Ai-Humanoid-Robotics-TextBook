# Physical AI & Humanoid Robotics — Interactive Textbook Implementation Plan

**Version**: 1.0 (Reverse Engineered)
**Date**: 2026-06-26

## Architecture Overview

**Architectural Style**: Modular monolith with two service-layer APIs (Connection + Agent)

**Reasoning**: Separation of concerns — Connection handles user-facing API, guardrails, and context resolution; Agent handles standalone RAG retrieval and LLM interaction. Both import from shared Retrieval module via `sys.path` hack.

## Layer Structure

### Layer 1: Frontend (Docusaurus + ChatKit)

- **Responsibility**: Render book content, provide chat UI with SSE streaming
- **Components**: ChatKit.tsx (SSE ReadableStream, localStorage, multi-session), Docusaurus theme components (Navbar swizzle, Footer swizzle, Sidebar, TOC), ModuleCards, HomepageFeatures
- **Dependencies**: → Connection API (port 8000/7860)
- **Technology**: Docusaurus 3.9.2, React 18, TypeScript 5.6, CSS modules

### Layer 2: Connection API (FastAPI, port 8000)

- **Responsibility**: Primary user-facing API — input validation, rate limiting, context resolution, guardrail check, agent orchestration, response formatting
- **Components**: `api.py` (routes), `agent_wrapper.py` (OpenRouter SDK calls), `guardrails.py` (`@input_guardrail`), `context_switcher.py` (selected_text vs Qdrant), `response_formatter.py` (source attribution)
- **Dependencies**: → Retrieval module (via `sys.path`), → OpenRouter API (external)
- **Technology**: Python 3.13, FastAPI, OpenAI Agents SDK, OpenAIChatCompletionsModel

### Layer 3: Agent API (FastAPI, port 8001)

- **Responsibility**: Standalone RAG agent — Qdrant retrieval, LLM interaction, streaming, session memory, health monitoring
- **Components**: `main.py` (routes), `agent.py` (RAGAgent with `process_message`/`process_message_streamed`), `adapter.py` (OpenRouterAdapter), `settings.py` (config), `validation.py` (response quality), `health_monitor.py`, `middleware.py` (rate limit), `logging_config.py`
- **Dependencies**: → Retrieval module (via `sys.path`), → OpenRouter API
- **Technology**: Python 3.13, FastAPI, OpenAI Agents SDK

### Layer 4: Retrieval Module (not a pip package)

- **Responsibility**: Qdrant vector search — embed queries via Cohere, search vector DB, return results
- **Components**: `client.py` (Qdrant client), `embedder.py` (Cohere), `search.py` (`search_similar`, `get_book_context`), `models.py`, `config.py`, `validation.py`, `latency.py`
- **Dependencies**: → Qdrant Cloud, → Cohere API
- **Technology**: Python, Qdrant client, Cohere client

### Layer 5: Ingestion Pipeline

- **Responsibility**: Ingest book markdown → chunk → embed → store in Qdrant
- **Components**: `chunker.py` (recursive markdown-aware splitting), `vectorizer.py`, `document_processor.py`, `text_cleaner.py`, `embedder.py`, `vector_store.py`
- **Dependencies**: → Cohere API, → Qdrant Cloud
- **Technology**: Python

## Data Flow

### Request Flow (Synchronous — POST /chat)

1. Frontend ChatKit sends `POST /chat` to Connection API
2. Connection validates input (empty, length, security patterns)
3. Rate limiter checks IP (100 req/min)
4. Session manager retrieves/creates `session_id`
5. ContextSwitcher resolves context: `selected_text` or Qdrant retrieval (`TOP_K=3`)
6. Guardrail checks if query is off-topic (LLM classification, `max_tokens=10`)
7. AgentWrapper builds instructions, calls OpenRouter via `OpenAIChatCompletionsModel` (`max_tokens=150`)
8. ResponseFormatter extracts sources from retrieved chunks
9. Session memory updated (last 5 turns)
10. Response returned to frontend

### Stream Flow (POST /chat/stream)

1–6. Same as synchronous
7. AgentWrapper calls OpenRouter with `run_streamed()`
8. Events streamed as SSE: each delta yields `{"token": "..."}`
9. On completion: `{"done": true, "session_id": "..."}`
10. If off-topic: `{"type": "off_topic", "content": "..."}`
11. Session memory updated after stream completes

## Technology Stack

| Layer | Technology |
|---|---|
| Frontend | Docusaurus 3.9.2, React 18, TypeScript 5.6, CSS modules |
| Backend | Python 3.13, FastAPI, uvicorn |
| AI SDK | OpenAI Agents SDK (`OpenAIChatCompletionsModel`, not Responses API) |
| LLM | OpenRouter (Qwen 2.5 72B Instruct) |
| Vector DB | Qdrant Cloud |
| Embeddings | Cohere (`embed-multilingual-v3.0`) |
| Package mgmt | `uv` workspace |
| Deployment (FE) | Vercel |
| Deployment (BE) | Docker / Hugging Face Spaces |

## Design Patterns Applied

### Pattern 1: Adapter Pattern

- **Location**: `Backend/Agent/adapter.py`
- **Purpose**: Abstract OpenRouter / OpenAI SDK behind a common interface (`create_agent`, `run_agent`, `run_agent_streamed`, `chat_completions_create`)
- **Implementation**: `OpenRouterAdapter` class wraps `AsyncOpenAI` client, Agent SDK `Runner`, and `ModelSettings`

### Pattern 2: Guardrail Pattern (SDK-native)

- **Location**: `Backend/Connection/guardrails.py` + `agent_wrapper.py`
- **Purpose**: Prevent off-topic queries from reaching the LLM
- **Implementation**: `@input_guardrail` decorator on `off_topic_guardrail` function, `GuardrailCtx` passed via `RunContext`

### Pattern 3: Context Switcher (Strategy-like)

- **Location**: `Backend/Connection/context_switcher.py`
- **Purpose**: Choose between user-selected text and Qdrant retrieval as context source
- **Implementation**: If `selected_text` exists, use it directly; otherwise call Qdrant

### Pattern 4: Session Memory (in-memory store)

- **Location**: `Backend/Connection/api.py` + `Backend/Agent/main.py`
- **Purpose**: Maintain conversation continuity across turns
- **Implementation**: `Dict[str, List[Dict]]` with UUID keys, TTL cleanup background task

### Pattern 5: Singleton

- **Location**: `Backend/Agent/adapter.py` (`agent_adapter` module-level instance), `Backend/Agent/settings.py` (`settings` module-level instance)
- **Purpose**: Single shared instance of client/configuration

### Pattern 6: Retry with Exponential Backoff

- **Location**: `Backend/Agent/adapter.py` (`chat_completions_create`), `Backend/Connection/agent_wrapper.py` (`_call_agent_with_retry`)
- **Purpose**: Resilience against transient OpenRouter failures
- **Implementation**: 3 attempts, 2^attempt sleep

## Module Breakdown

### Module: Connection API

| Attribute | Value |
|---|---|
| **Purpose** | Primary user-facing chat API |
| **Key Files** | `api.py`, `agent_wrapper.py`, `guardrails.py`, `context_switcher.py`, `response_formatter.py`, `models.py`, `config.py`, `main.py` |
| **Dependencies** | Retrieval module (`sys.path`), OpenRouter API, Qdrant |
| **Complexity** | M |

### Module: Agent API

| Attribute | Value |
|---|---|
| **Purpose** | Standalone RAG agent |
| **Key Files** | `main.py`, `agent.py`, `adapter.py`, `settings.py`, `validation.py`, `health_monitor.py`, `middleware.py`, `logging_config.py` |
| **Dependencies** | Retrieval module (`sys.path`), OpenRouter API, Qdrant |
| **Complexity** | M |

### Module: Retrieval

| Attribute | Value |
|---|---|
| **Purpose** | Qdrant vector search |
| **Key Files** | `search.py`, `client.py`, `embedder.py`, `models.py`, `config.py`, `validation.py`, `latency.py` |
| **Dependencies** | Qdrant Cloud, Cohere API |
| **Complexity** | L |

### Module: Ingestion

| Attribute | Value |
|---|---|
| **Purpose** | Book → Qdrant pipeline |
| **Key Files** | `chunker.py`, `vectorizer.py`, `document_processor.py`, `text_cleaner.py`, `embedder.py`, `vector_store.py` |
| **Dependencies** | Cohere API, Qdrant Cloud |
| **Complexity** | M |

### Module: ChatKit UI

| Attribute | Value |
|---|---|
| **Purpose** | In-browser chat assistant with SSE, localStorage persistence |
| **Key Files** | `ChatKit.tsx`, `ChatKit.css` |
| **Dependencies** | Connection API |
| **Complexity** | M |

### Module: Docusaurus Theme

| Attribute | Value |
|---|---|
| **Purpose** | Customized textbook site with glassmorphism dark theme |
| **Key Files** | `custom.css`, `index.module.css`, `DocSidebar/styles.module.css`, `TOC/styles.module.css`, `Footer/styles.module.css`, `Navbar/styles.module.css` |
| **Dependencies** | Docusaurus core |
| **Complexity** | L |

## Regeneration Strategy

**Recommended**: Incremental refactoring — use existing specs as foundation, address technical debt, and improve test coverage.

## Improvement Opportunities

- [ ] Package Retrieval as proper pip package (remove `sys.path` hack)
- [ ] Add integration tests for Connection API
- [ ] Add structured logging (JSON format)
- [ ] Restrict CORS in production
- [ ] Add session persistence (Redis) for horizontal scaling
- [ ] Remove dead code (`error_handling.py`, 12 unreferenced images, 3 unused sim files)
- [ ] Reconnect frontend assets to meaningful content (sidebar URLs, footer links)
- [ ] Add OpenAPI/Swagger documentation
- [ ] Add CI/CD pipeline (GitHub Actions)
