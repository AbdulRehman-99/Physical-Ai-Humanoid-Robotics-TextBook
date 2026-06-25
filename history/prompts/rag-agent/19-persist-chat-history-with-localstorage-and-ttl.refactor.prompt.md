---
id: 19
title: Persist Chat History with localStorage and TTL
stage: refactor
date: 2026-06-25
surface: agent
model: deepseek-v4-flash-free
feature: rag-agent
branch: main
user: user
command: custom
labels: [persistence,localstorage,session,ttl,history]
links:
  spec: null
  plan: null
  tasks: null
  ticket: null
  adr: null
  pr: null
files:
 - Backend/Connection/config.py
 - Backend/Connection/api.py
 - Backend/Agent/main.py
 - Frontend/src/components/ChatKit/ChatKit.tsx
tests:
 - (none)
---

# Persist Chat History with localStorage and TTL

## Summary
Refactored chat history management to persist across page refreshes:
- Frontend: sessionId and messages persisted in localStorage with 100-message cap
- Backend: TTL-based session eviction, uuid4 IDs, configurable turn limit
- Fixed Agent service bug where POST /chat ignored sessions entirely

## Changes

### Frontend: `ChatKit.tsx`
- Added `loadMessages()` — restores messages from `localStorage` key `chatkit_messages` on mount; falls back to welcome message if empty
- Added `saveMessages()` — debounced (500ms) save on every messages change; caps at `MAX_MESSAGES = 100`
- Changed sessionId generation from `session_${Date.now()}` → `crypto.randomUUID()` persisted to `localStorage` key `chatkit_session_id`
- When backend returns a new `session_id` in streaming response, saves to localStorage
- All operations wrapped in try/catch to handle private browsing / storage-full scenarios

### Backend: `Connection/config.py`
- Added `SESSION_MEMORY_TURNS = 5` (replaces hardcoded `5` in api.py)
- Added `SESSION_TTL_SECONDS = 1800` (30-minute session idle timeout)

### Backend: `Connection/api.py`
- Added `session_last_access: Dict[str, float]` to track per-session last activity
- `_get_or_create_session`: uses `uuid.uuid4()` instead of `session_{len}_{time.time()}`; updates `session_last_access`
- `_update_memory`: uses `SESSION_MEMORY_TURNS` config value; updates `session_last_access`
- Added `_session_cleanup_task()` — background asyncio task running every 5 minutes, evicts sessions idle > `SESSION_TTL_SECONDS`
- Added `@app.on_event("startup")` to launch cleanup task

### Backend: `Agent/main.py` (bug fix)
- Added `session_id: Optional[str]` to `ChatPayload` (was missing — caused POST /chat to ignore sessions)
- Added `_get_or_create_session()` and `_update_memory()` calls in the POST /chat endpoint
- Passes `memory` to `agent.process_message()`

## Key Behaviors
- Page refresh: sessionId loaded from localStorage → backend recognizes session → memory restored
- Page refresh: messages restored from localStorage → full chat history visible
- Session timeout: after 30min idle, backend evicts session → next request gets fresh session
- Storage protection: all localStorage calls wrapped in try/catch to handle quota exceeded or private browsing

## Testing
- Python syntax verified for all 3 backend files
