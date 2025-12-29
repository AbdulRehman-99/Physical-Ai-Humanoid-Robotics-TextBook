---
id: 3
title: Plan RAG Agent Implementation
stage: plan
date: 2025-12-23
surface: agent
model: claude-sonnet-4-5-20250929
feature: rag-agent
branch: main
user: user
command: /sp.plan
labels: [plan,rag,agent,book-content,fastapi]
links:
  spec: specs/003-rag-agent/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/003-rag-agent/plan.md
 - specs/003-rag-agent/research.md
 - specs/003-rag-agent/data-model.md
 - specs/003-rag-agent/quickstart.md
 - specs/003-rag-agent/contracts/chat-api.yaml
tests:
 -
---

## Prompt

Goal: Design and implement a FastAPI-based RAG backend using OpenAI Agents SDK v0.6, and Chatkit UI with Cohere embeddings and Qdrant database..

Architecture:
- **Core Framework:** FastAPI
- **Agent Logic:** OpenAI Agents SDK v0.6
- **Chat UI:** Chatkit SDK
- **LLM Layer:** Gemini Flash/Pro accessed via `LiteLLM` proxy to mimic OpenAI compatibility
- **Data Layer:** Existing Qdrant Collection + Cohere Embeddings (Ported from Step 2)
- **Package Manager:** uv (for Python 3.13 management and fast dependency resolution)

Technical Plan:

Step 1: Project Initialization & Dependencies
- Install `uv` and install all required packages or dependencies using a `uv add`.
- Initialize the project specifically with Python 3.13: `uv init --python 3.13 Backend/Agent` and navigate into it.
- Create folder structure: `Backend/Agent/` containing `main.py`, `agent.py`, `adapter.py`, `retrieval.py`, `settings.py`.
- Create `requirements.txt`: Include `fastapi`, `uvicorn`, `openai-agents`, `litellm`, `cohere`, `qdrant-client`, `pydantic`.
- Set the GEMINI_API_KEY into the .env file that is present in the root of the project..

Step 2: LLM Adapter Implementation (Crucial)
- Create `app/adapter.py`.
- Implement a custom client factory that routes OpenAI SDK requests to Google Gemini using `LiteLLM` or a custom `base_url`.
- **Validation:** Write a script `test_adapter.py` to ensure the Agent SDK can successfully handshake with Gemini.

Step 3: Retrieval Module Integration
- Create `app/retrieval.py`.
- Port the validated retrieval logic from Step 2 into a reusable function: `get_book_context(query: str, top_k: int=5) -> str`.
- Ensure this function handles embedding generation (Cohere) and vector search (Qdrant) seamlessly.

Step 4: Agent & Context Orchestration
- Create `app/agent.py`.
- Define the `Agent` class using the SDK's `Worker` pattern.
- Implement the **Context Switcher Logic**:
  - **Scenario A (Selected Text):** If `selected_text` provided -> System Prompt uses ONLY that text.
  - **Scenario B (General Query):** If `selected_text` is null -> Call `get_book_context` -> System Prompt uses retrieved chunks.
- **Guardrails:** Inject system instructions to refuse off-topic queries and strictly adhere to the provided context.

Step 5: FastAPI Endpoint & ChatKit Interface
- Create `app/main.py`.
- Define Pydantic Schema: `class ChatPayload(BaseModel): message: str, selected_text: Optional[str]`.
- Implement `POST /chat`:
  - Accept JSON payload.
  - Invoke the Agent.
  - Return response in a format compatible with ChatKi

## Response snapshot

Completed implementation planning for the RAG Agent feature, creating all required planning artifacts:
1. Implementation plan (plan.md) with technical context and architecture
2. Research document (research.md) with technical decisions and alternatives
3. Data model (data-model.md) defining all entities and relationships
4. Quickstart guide (quickstart.md) with setup and usage instructions
5. API contract (contracts/chat-api.yaml) in OpenAPI format

The plan addresses all requirements from the prompt including FastAPI backend, OpenAI Agents SDK v0.6, Cohere embeddings, Qdrant database, and the context switching logic based on selected_text parameter.

## Outcome

- ✅ Impact: Comprehensive planning artifacts created that align with all architectural requirements
- 🧪 Tests: All artifacts are well-defined and testable
- 📁 Files: Created plan.md, research.md, data-model.md, quickstart.md, and API contract
- 🔁 Next prompts: Ready for task generation with /sp.tasks
- 🧠 Reflection: Planning addresses all architectural requirements while maintaining flexibility for implementation

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable):
- Next experiment (smallest change to try): Proceed to task generation phase