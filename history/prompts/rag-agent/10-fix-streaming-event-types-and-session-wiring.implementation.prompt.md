---
id: 10
title: Fix Streaming Event Types and Session Wiring
stage: refactor
date: 2026-06-25
surface: agent
model: deepseek-v4-flash-free
feature: rag-agent
branch: main
user: user
command: custom
labels: [bugfix,streaming,session,history-toggle]
links:
  spec: null
  plan: null
  tasks: null
  ticket: null
  adr: null
  pr: null
files:
 - Backend/Connection/agent_wrapper.py
 - Backend/Agent/adapter.py
 - Backend/Connection/models.py
 - Backend/Connection/api.py
 - Frontend/src/components/ChatKit/ChatKit.tsx
 - Frontend/src/components/ChatKit/ChatKit.css
tests:
 - Backend/Ingestion/tests/test_chunker.py
---

# Fix Streaming Event Types, Session Wiring, and Add History Toggle

## Summary
Fixed three issues: (1) streaming was not displaying in real-time because the backend used wrong event type/attribute from the OpenAI Agents SDK, (2) conversation message history was broken because `session_id` was not wired through the request pipeline, (3) added a "Show all" / "Show last 6" toggle button to the frontend.

## Root Causes & Fixes

### Issue 1: Streaming Not Displaying in Real-Time
- **Root cause:** Both `agent_wrapper.py` and `adapter.py` checked `event.data.type == "chunk"` and `event.data.content`, but the Agents SDK v0.17.x emits `ResponseTextDeltaEvent` with `type == "response.output_text.delta"` and content in `.delta`.
- **Fix:** Changed to `getattr(event.data, 'type', None) == "response.output_text.delta"` and read `event.data.delta`.

### Issue 2: Message History Not Loading
- **Root cause:** `ChatRequest` had no `session_id` field, and both `/chat` and `/chat/stream` endpoints called `_get_or_create_session(None)`, generating a new session on every request.
- **Fix:** Added `session_id: Optional[str] = None` to `ChatRequest`. Both endpoints now pass `chat_request.session_id` to `_get_or_create_session()`. Frontend sends `session_id` in POST body too.

### Issue 3: History Toggle Button
- **Added** `showHistory` state, `displayMessages` computed list (all messages vs last 6), and a toggle button in the `chatkit-history-bar`. On refresh, state resets (messages cleared). CSS added for the toggle button.

## Files Modified
- `Backend/Connection/agent_wrapper.py:74-77` — fixed event type check
- `Backend/Agent/adapter.py:102-105` — fixed event type check
- `Backend/Connection/models.py:9` — added `session_id` field
- `Backend/Connection/api.py:122,160` — wired `session_id` in both endpoints
- `Frontend/src/components/ChatKit/ChatKit.tsx` — send `session_id` in POST, added `showHistory` toggle + `displayMessages` filtering
- `Frontend/src/components/ChatKit/ChatKit.css` — added `.chatkit-history-bar` and `.chatkit-history-toggle` styles

## Testing
- All 8 chunker tests pass (regression)
