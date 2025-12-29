# Implementation Plan: RAG Agent for Book Content

**Branch**: `003-rag-agent` | **Date**: 2025-12-23 | **Spec**: specs/003-rag-agent/spec.md
**Input**: Feature specification from `/specs/[003-rag-agent]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a FastAPI-based RAG backend using OpenAI Agents SDK v0.6, with Chatkit UI integration and Cohere embeddings with Qdrant database. The system will handle user queries about book content with strict grounding in provided context, supporting both general queries and queries based on selected text.

## Technical Plan

### Step 1: Project Initialization & Dependencies
- Install `uv` and install all required packages or dependencies using a `uv add`.
- Initialize the project specifically with Python 3.13: `uv init --python 3.13 Backend/Agent` and navigate into it.
- Create folder structure: `Backend/Agent/` containing `main.py`, `agent.py`, `adapter.py`, `retrieval.py`, `settings.py`.
- Create `pyproject.toml`: Include `fastapi`, `uvicorn`, `openai-agents`, `litellm`, `cohere`, `qdrant-client`, `pydantic`.
- Set the GEMINI_API_KEY into the .env file that is present in the root of the project.

### Step 2: LLM Adapter Implementation (Crucial)
- Create `app/adapter.py`.
- Implement a custom client factory that routes OpenAI SDK requests to Google Gemini using `LiteLLM` with specific configuration:
  - Use `gemini/gemini-pro` or `gemini/gemini-flash` model identifiers
  - Set base URL to `https://generativelanguage.googleapis.com/v1beta`
  - Configure proper API key handling via environment variables
  - Map OpenAI Agent SDK v0.6 interface to Gemini API format
- **Validation:** Write a script `test_adapter.py` to ensure the Agent SDK can successfully handshake with Gemini.

### Step 3: Retrieval Module Integration
- Create `app/retrieval.py`.
- Port the validated retrieval logic from Step 2 into a reusable function: `get_book_context(query: str, top_k: int=5) -> str`.
- Ensure this function handles embedding generation (Cohere) and vector search (Qdrant) seamlessly.

### Step 4: Agent & Context Orchestration
- Create `app/agent.py`.
- Define the `Agent` class using the SDK's `Worker` pattern.
- Implement the **Context Switcher Logic**:
  - **Scenario A (Selected Text):** If `selected_text` provided -> System Prompt uses ONLY that text.
  - **Scenario B (General Query):** If `selected_text` is null -> Call `get_book_context` -> System Prompt uses retrieved chunks.
- **Guardrails:** Inject system instructions to refuse off-topic queries and strictly adhere to the provided context.

### Step 5: FastAPI Endpoint & ChatKit Interface
- Create `app/main.py`.
- Define Pydantic Schema: `class ChatPayload(BaseModel): message: str, selected_text: Optional[str]`.
- Implement `POST /chat`:
  - Accept JSON payload.
  - Invoke the Agent.
  - Return response in a format compatible with ChatKit.

## Technical Context

**Language/Version**: Python 3.13
**Primary Dependencies**: FastAPI, OpenAI Agents SDK v0.6, LiteLLM, Cohere, Qdrant-client, uv
**Storage**: Qdrant Cloud vector database (for book content), in-memory processing for user queries
**Testing**: pytest for backend functionality and integration tests
**Target Platform**: Linux server deployment
**Project Type**: Web backend service
**Performance Goals**: Support 100 concurrent users, respond to queries within 5 seconds for 95% of requests
**Constraints**: <200ms p95 for internal operations, <10,000 char limit for selected text, no persistent storage of user queries
**Scale/Scope**: 100 concurrent users, ~1k-5k book content chunks in vector database

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

The implementation aligns with the High-Performance Technical Book Constitution by:
- Following academic reliability principles for the RAG system
- Using precise citations and verified sources from book content
- Ensuring reproducible and clear system design
- Excluding speculation by strictly grounding responses in provided context
- Following approved toolchain for the backend implementation

## Project Structure

### Documentation (this feature)

```text
specs/003-rag-agent/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
Backend/Agent/
├── main.py              # FastAPI application entry point
├── agent.py             # OpenAI Agent SDK implementation
├── adapter.py           # LLM adapter for Gemini via LiteLLM
├── retrieval.py         # Qdrant + Cohere retrieval logic
├── settings.py          # Configuration and settings
├── pyproject.toml       # Python dependencies managed with uv
└── .env                 # Environment variables
```

**Structure Decision**: Web backend service with FastAPI for the RAG agent functionality, following the multi-project structure with dedicated Backend/Agent directory for the agent implementation.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |