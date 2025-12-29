---
id: 1
title: Create RAG Agent Spec
stage: spec
date: 2025-12-23
surface: agent
model: claude-sonnet-4-5-20250929
feature: rag-agent
branch: main
user: user
command: /sp.specify
labels: [spec,rag,agent,book-content]
links:
  spec: specs/003-rag-agent/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/003-rag-agent/spec.md
 - specs/003-rag-agent/checklists/requirements.md
tests:
 -
---

## Prompt

Create a new inside the specs for Spec-3: Agent

Context:
- Frontend: Docusaurus with the official ChatKit SDK:  https://platform.openai.com/docs/guides/chatkit
- Backend: FastAPI:  https://fastapi.tiangolo.com/
- Agent Framework: OpenAI Agents SDK (Python):  https://openai.github.io/openai-agents-python/
- LLM: Gemini (used via an OpenAI Agents SDK–compatible adapter)
- Embeddings: Cohere
- Vector Database: Qdrant Cloud
- Use Python as a main language
- Book content has already been ingested and is retrievable

Objective:
- Build a backend RAG agent that answers user questions strictly from book content or user-selected text, with no external knowledge.

Functional Requirements:
- Endpoint: Expose a single POST /chat endpoint compatible with ChatKit.-
- Payload: Accepts { message: string, selected_text: string | null }.

Context Logic:
- If selected_text exists: Agent MUST answer strictly from that text and ignore vector retrieval.
- If selected_text is null: Embed query (Cohere), retrieve top-K chunks (Qdrant), and use as context.

Guardrails:
- Agent must answer strictly from the provided context (book or selection).
- Safely refuse all off-topic/external knowledge queries without throwing HTTP errors.

Hard Constraints:
- The chat UI MUST use the official ChatKit SDK only. No custom chat UI components are allowed, No alternative chat libraries are allowed
- ChatKit is responsible only for UI rendering and message transport
- ChatKit MUST send user messages to a FastAPI POST endpoint
- Use ONLY the official `openai-agents` Python package (version 0.6), Do NOT use the legacy `openai` Python package
- All agent creation, execution, and context management MUST follow the official OpenAI Agents SDK v0.6 documentation: https://openai.github.io/openai-agents-python/
- Python version 3.13
- No ingestion or chunking logic

Success Criteria:
- Book-related questions return accurate, grounded answers
- Selected-text queries override vector retrieval
- Off-topic queries are consistent

## Response snapshot

Created a comprehensive specification for the RAG agent feature that includes user scenarios, functional requirements, key entities, and success criteria. The spec defines a system that exposes a POST /chat endpoint compatible with ChatKit, accepts messages with optional selected text, and ensures responses are grounded strictly in book content or user-selected text. The specification includes proper handling of off-topic queries and integration with the required technology stack.

## Outcome

- ✅ Impact: Created complete specification for RAG agent feature with user stories, requirements, and success criteria
- 🧪 Tests: Specification includes testable acceptance scenarios for each user story
- 📁 Files: Created spec.md and requirements checklist in specs/003-rag-agent/
- 🔁 Next prompts: Ready for planning phase with /sp.plan
- 🧠 Reflection: Specification properly captures the RAG agent requirements with appropriate constraints and success metrics

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable):
- Next experiment (smallest change to try): Proceed to planning phase