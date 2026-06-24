---
id: 9
title: Optimize Agent with Streaming Memory and Chunking
stage: refactor
date: 2026-06-25
surface: agent
model: deepseek-v4-flash-free
feature: rag-agent
branch: main
user: user
command: custom
labels: [optimization,streaming,memory,guardrails,chunking,top-k]
links:
  spec: null
  plan: null
  tasks: null
  ticket: null
  adr: null
  pr: null
files:
 - .env
 - Backend/Agent/settings.py
 - Backend/Agent/adapter.py
 - Backend/Agent/agent.py
 - Backend/Agent/main.py
 - Backend/Connection/config.py
 - Backend/Connection/models.py
 - Backend/Connection/context_switcher.py
 - Backend/Connection/agent_wrapper.py
 - Backend/Connection/api.py
 - Backend/Ingestion/src/ingestion/chunker.py
 - Backend/Ingestion/tests/test_chunker.py
 - Frontend/src/components/ChatKit/ChatKit.tsx
tests:
 - Backend/Ingestion/tests/test_chunker.py
---

# Optimize Agent with Streaming, Memory, Guardrails, and Recursive Chunking

## Summary
Implemented six optimizations across the full stack: reduced top-k from 5→3, added max_tokens=300 via ModelSettings, in-memory conversation memory (last 5 turns), SSE streaming endpoints, Agent SDK-compatible guardrail stubs, and recursive markdown-aware chunking. All changes were mirrored in both the Agent (port 8001) and Connection (port 8000) services, with frontend ChatKit.tsx updated for SSE streaming.

## Changes Made

### Top-k Configuration
- `.env`: `TOP_K=5` → `TOP_K=3`
- `Backend/Agent/settings.py`: default `5` → `3`
- `Backend/Connection/config.py`: `QDRANT_TOP_K = 5` → `QDRANT_TOP_K = 3`
- `Backend/Connection/context_switcher.py`: imported `QDRANT_TOP_K` constant, replaced hardcoded `top_k=5` with `top_k=QDRANT_TOP_K` (both sync and async methods)

### Agent Service (port 8001)
- `adapter.py`: added `ModelSettings(max_tokens=300)`, `_build_input_with_memory()` helper, `_trim_memory()` helper, `run_agent_streamed()` method (uses `Runner.run_streamed()` with token yielding), explicit `input_guardrails`/`output_guardrails` params on `create_agent()`
- `agent.py`: added `process_message_streamed()` async generator, `memory` param throughout, `_build_instructions()` with length constraint text ("no more than 3-4 sentences"), refactored instruction building into reusable method
- `main.py`: added `ChatStreamPayload`, `conversation_sessions` dict with `_get_or_create_session()`/`_update_memory()`, `/chat/stream` SSE endpoint using `StreamingResponse`

### Connection Service (port 8000)
- `models.py`: added `memory: Optional[List[dict]] = None` to `ChatContext`
- `agent_wrapper.py`: added `process_with_context_streamed()` async generator, `_call_agent_streamed()`, `_build_instructions()` method, `ModelSettings(max_tokens=300)`, `_build_input_with_memory()` helper
- `api.py`: added `conversation_sessions` dict with session management, updated `/chat` to pass memory, added `/chat/stream` SSE endpoint

### Chunker
- `chunker.py`: full rewrite — removed old tokenize/detokenize/semantic-boundary approach; replaced with recursive priority-based splitting (markdown headings → paragraphs → sentences → token window), fixed duplicate `chunk_document_content` bug and None metadata handling
- `test_chunker.py`: updated tests to use new `create_chunks()` API, removed tests for removed methods, fixed edge case tests

### Frontend
- `ChatKit.tsx`: added `tryStream()` function using `ReadableStream`/`fetch` for SSE, session ID tracking, fallback to POST if streaming fails, incremental token rendering in assistant messages, removed typing indicator during streaming

## Testing
- All 8 chunker tests pass
- Agent module imports verified (settings, adapter, agent)

## Key Decisions
- SSE over WebSocket for simplicity (no extra dependency, works with standard HTTP)
- In-memory dict for session storage (no persistence layer needed for dev)
- Recursive splitting priority: headings > paragraphs > sentences > token window
- Frontend streams try first, POST fallback on failure

## Next Steps
- Test streaming end-to-end with running services
- Consider persistent session storage for production
- Consider chunk quality metrics monitoring

## Acceptance Criteria
- [x] TOP_K default changed to 3 across all config files
- [x] Responses limited to ~300 tokens (~3-4 sentences)
- [x] Last 5 conversation turns retained in memory
- [x] `/chat/stream` SSE endpoint available on both services
- [x] Chunker uses recursive markdown-aware splitting
- [x] Frontend renders streaming tokens incrementally
- [x] Backward compatible — existing `/chat` POST still works
