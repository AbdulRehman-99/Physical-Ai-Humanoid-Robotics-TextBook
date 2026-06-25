# Physical AI & Humanoid Robotics — Interactive Textbook

A full-stack interactive textbook platform for **Physical AI & Humanoid Robotics**, combining a Docusaurus documentation site with an AI-powered RAG (Retrieval-Augmented Generation) chat assistant. Users can read the book and ask questions about its content through an embedded chat interface.

## Features

- **Interactive Book** — Docusaurus site with 4 modules covering ROS 2, Gazebo simulation, NVIDIA Isaac, and advanced integration
- **AI Chat Assistant** — ChatKit component embedded in every page, answers questions grounded in the book content
- **SSE Streaming** — Real-time token streaming from the LLM via Server-Sent Events
- **Session Memory** — Last 5 conversation turns per session (in-memory, 30-min TTL)
- **Off-Topic Guardrails** — Dual-layer LLM-based guardrail prevents unrelated queries
- **Multi-Session Chat History** — Frontend localStorage with 48h auto-delete
- **Vector Search** — Qdrant vector database with Cohere embeddings for semantic retrieval (TOP_K=3)
- **Futuristic UI** — Glassmorphism dark theme with Midnight Blue/Electric Cyan palette

## Architecture

```
Frontend/              Docusaurus 3.9.2, React 18, TypeScript
  └─ ChatKit/           SSE streaming chat, localStorage history
Backend/
  ├─ Connection/        Primary API (port 8000) — consumed by ChatKit
  │   ├─ api.py          FastAPI: POST /chat, POST /chat/stream, GET /health
  │   ├─ agent_wrapper   OpenRouter via OpenAI Agent SDK, off-topic guardrails
  │   ├─ context_switcher sys.path-hacks Retrieval/src/
  │   ├─ guardrails      @input_guardrail for off-topic detection
  │   └─ config          TOP_K=3, SESSION_TTL=1800s, memory=5 turns
  ├─ Agent/             Standalone RAG Agent (port 8001)
  │   ├─ main.py         /chat + /chat/stream + /health + /health/database
  │   ├─ agent.py        RAGAgent with streaming + memory + guardrail
  │   └─ adapter.py      OpenRouterAdapter (shared client pattern)
  ├─ Ingestion/         Book → Qdrant ingestion pipeline (Cohere embeddings)
  ├─ Retrieval/         Qdrant vector search (sys.path-imported, not a pip package)
  ├─ main.py            Orchestrator: starts all 3 processes
  └─ app.py             Hugging Face Spaces entry point (port 7860)
```

**Key design:**
- Both services use **OpenRouter** via `OpenAIChatCompletionsModel` (not the Responses API — OpenRouter ignores `instructions` on `/v1/responses`)
- `max_tokens=150` everywhere (targets 80–100 word answers)
- Session memory: in-memory dict, UUID session IDs, 30-min idle TTL
- Retrieval module is imported via `sys.path` hack (not a pip package)
- Plain-text formatting only (no markdown symbols), inline colon labels for headers

## Prerequisites

- **Node.js** >= 20.0
- **Python** == 3.13
- **uv** (Python package manager) — install from https://docs.astral.sh/uv/

## Setup

### 1. Clone and environment

```bash
# Copy the example environment file
cp .env.example .env   # or create .env manually (see below)
```

Required environment variables (set in `.env` at repo root):

| Variable | Required | Default | Purpose |
|---|---|---|---|
| `OPENROUTER_API_KEY` | Yes | — | LLM provider |
| `OPENROUTER_BASE_URL` | No | `https://openrouter.ai/api/v1` | OpenRouter endpoint |
| `OPENROUTER_MODEL` | No | `qwen-2.5-72b-instruct` | LLM model |
| `COHERE_API_KEY` | Yes | — | Embeddings |
| `QDRANT_API_KEY` | Yes | — | Vector DB |
| `QDRANT_URL` | Yes | — | Qdrant cloud URL |
| `COLLECTION_NAME` | No | `book_embeddings` | Qdrant collection |
| `TOP_K` | No | `3` | Retrieved chunks |

### 2. Frontend

```bash
cd Frontend
npm install
```

### 3. Backend

```bash
cd Backend
uv sync        # install Python deps from pyproject.toml
```

## Running

### Development (all services)

```powershell
# Frontend dev server
cd Frontend; npm start

# Backend orchestrator (Connection:8000 + Agent:8001 + Retrieval test)
cd Backend; uv run python main.py

# Or hot-reload (Connection only)
$env:DEBUG="True"; cd Backend; uv run python app.py
```

### Individual services

```bash
# Connection API only (port 8000)
cd Backend; uv run python -m uvicorn Connection.api:app --port 8000 --reload

# Agent API only (port 8001)
cd Backend; uv run python -m uvicorn Agent.main:app --port 8001 --reload
```

### Tests

```bash
cd Backend/Agent; pytest
```

### Type checking

```bash
cd Frontend; npm run typecheck
```

## Usage

### Chat API

Send a POST request to `http://localhost:8000/chat`:

```json
{
  "message": "What is embodied intelligence?",
  "session_id": "optional-uuid-for-conversation-memory"
}
```

Response (200):
```json
{
  "response": "Based on the provided textbook content, Embodied Intelligence is:\n- A concept where intelligence arises from agent-environment interaction\n- Rooted in the idea that cognition requires a physical body\n- Central to humanoid robotics design...",
  "sources": ["Module: 1, Chapter: 1, Section: 1.1"],
  "is_off_topic": false
}
```

### Streaming Chat

POST to `http://localhost:8000/chat/stream` with the same body. Response is an SSE stream:

```
{"token": "Based"}
{"token": " on"}
{"token": " the"}
...
{"done": true, "session_id": "uuid"}
```

If the query is off-topic, the first (and only) event is:
```
{"type": "off_topic", "content": "I can only answer questions about the book content..."}
```

### Health Check

```
GET http://localhost:8000/health
→ {"status": "healthy", "timestamp": 1234567890.0}
```

## Project Structure

```
./
├── Frontend/                  Docusaurus site
│   ├── docs/                  Book content (8 chapters across 4 modules)
│   ├── src/
│   │   ├── components/ChatKit/ Chat UI (ChatKit.tsx, ChatKit.css)
│   │   ├── css/custom.css     Global styles (glassmorphism dark theme)
│   │   ├── pages/             Homepage, custom pages
│   │   └── theme/             Swizzled Docusaurus theme components
│   ├── static/img/            Images (3 referenced, 12 unreferenced)
│   └── assets/                Gazebo/URDF sim files (unused in code)
│
├── Backend/                   Python backend (uv workspace)
│   ├── Connection/            Primary user-facing API (port 8000)
│   ├── Agent/                 RAG Agent (port 8001)
│   ├── Ingestion/             Book → Qdrant pipeline
│   ├── Retrieval/             Vector search service
│   ├── main.py                Process orchestrator
│   ├── app.py                 HF Spaces entry point
│   └── Dockerfile             Production container
│
├── specs/                     Architecture specs (SDD workflow)
├── history/                   PHRs + ADRs
├── .specify/                  SpecKit Plus templates
├── .opencode/                 opencode command definitions
├── AGENTS.md                  Project + workflow instructions
└── opencode.md                → delegates to AGENTS.md
```

## Deployment

### Frontend (Vercel)

```bash
# vercel.json handles this automatically:
cd Frontend && npm install && npm run build
# Output: Frontend/build/
```

### Backend (Hugging Face Spaces)

Dockerfile at `Backend/Dockerfile`, runs `python app.py` on port 7860.

### Backend URL in Frontend

Hardcoded in two places:
- `Frontend/src/components/ChatKit/ChatKit.tsx:14-18` — `BACKEND_URLS` array (fallback chain)
- `Frontend/docusaurus.config.ts:31` — `customFields.backendUrl` (for SSR)

## Tech Stack

| Layer | Technology |
|---|---|
| Frontend framework | Docusaurus 3.9.2, React 18, TypeScript 5.6 |
| Backend framework | Python 3.13, FastAPI, uvicorn |
| LLM provider | OpenRouter (Qwen 2.5 72B Instruct) |
| AI SDK | OpenAI Agents SDK (`OpenAIChatCompletionsModel`) |
| Vector database | Qdrant Cloud |
| Embeddings | Cohere (`embed-multilingual-v3.0`) |
| Python package mgmt | uv workspace |
| Deployment (FE) | Vercel |
| Deployment (BE) | Docker / Hugging Face Spaces |

## Notable unused / dead code

- `Backend/Agent/error_handling.py` — CircuitBreaker + Qdrant error decorator, only imported by tests
- `Frontend/static/img/` — 12 SVG/PNG files with zero source references (including `favicon.ico`, `logo.svg`, 3 Docusaurus defaults)
- `Frontend/assets/` — 3 Gazebo/URDF sim files not referenced in any code
