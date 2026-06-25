---
id: 18
title: Add Input Guardrail for Off-Topic Detection
stage: implementation
date: 2026-06-25
surface: agent
model: deepseek-v4-flash-free
feature: rag-agent
branch: main
user: user
command: custom
labels: [guardrail,off-topic,input-guardrail,run-context,error-ui]
links:
  spec: null
  plan: null
  tasks: null
  ticket: null
  adr: null
  pr: null
files:
 - Backend/Connection/guardrails.py
 - Backend/Connection/models.py
 - Backend/Connection/agent_wrapper.py
 - Backend/Connection/api.py
 - Frontend/src/components/ChatKit/ChatKit.tsx
 - Frontend/src/components/ChatKit/ChatKit.css
tests:
 - (none)
---

# Add Input Guardrail for Off-Topic Detection

## Summary
Implemented an OpenAI Agent SDK `@input_guardrail` that checks whether a user's question is answerable from the retrieved book context before the agent runs. If off-topic, the frontend displays a visually distinct error message instead of a normal assistant response.

## Architecture

```
User query → api.py → context_switcher (retrieves book content)
  → agent_wrapper (creates Agent with input_guardrail)
    → SDK runs off_topic_guardrail (LLM checks if query relates to book content)
      → tripwire_triggered=True  → api.py sends `{"type": "off_topic"}` → UI shows error banner
      → tripwire_triggered=False → agent runs normally → UI shows answer
```

## Changes

### New file: `Backend/Connection/guardrails.py`
- `GuardrailCtx` — plain class (not pydantic — AsyncOpenAI client is not serializable) holding `book_content`, `client`, `model_name` passed via `RunContext`
- `_check_off_topic()` — core check: makes a max_tokens=10 LLM call to classify the question as answerable from the book content
- `off_topic_guardrail` — `@input_guardrail` decorated function that wraps `_check_off_topic` for the SDK pipeline

### `Backend/Connection/models.py`
- Added `is_off_topic: bool = False` to `ChatResponse`

### `Backend/Connection/agent_wrapper.py`
- Added imports: `RunContextWrapper`, `GuardrailCtx`, `off_topic_guardrail`, `_check_off_topic`
- Added `OFF_TOPIC_SENTINEL = "__OFF_TOPIC__"` constant
- `_call_agent_streamed()`: runs `_check_off_topic()` manually before streaming; yields `OFF_TOPIC_SENTINEL` if off-topic
- `_call_agent_internal()`: passes `GuardrailCtx` via `RunContext`, sets `input_guardrails=[off_topic_guardrail]`, checks `result.guardrail_result.tripwire_triggered` after `Runner.run()`
- Removed keyword-based off-topic detection (lines 209-214)

### `Backend/Connection/api.py`
- Imported `OFF_TOPIC_SENTINEL` from `agent_wrapper`
- POST endpoint: checks for sentinel after `process_with_context()`, returns `ChatResponse(is_off_topic=True)`
- Streaming endpoint: checks first token for sentinel in token_generator, sends `{"type": "off_topic"}` SSE event

### `Frontend/ChatKit.tsx`
- Added `isError?: boolean` to `Message` interface
- Streaming: handles `data.type === "off_topic"` — replaces blank assistant message with error message
- POST: handles `data.is_off_topic` — adds message with `isError: true`
- Render: adds `.error` CSS class when `message.isError`

### `Frontend/ChatKit.css`
- Added `.chatkit-message.error .chatkit-message-content` — red-tinted glass background, red border
- Added `::before` pseudo-element with "!" warning icon on the left

## Key Design Decisions
- Streaming path runs guardrail manually (not through SDK pipeline) because SDK guardrail events in streamed mode are unreliable for this use case
- Guardrail uses the SAME OpenRouter client and model as the agent (reuses connection pool)
- Guardrail LLM call costs ~10 tokens per query (max_tokens=10)
- `GuardrailCtx` is passed via `RunContext` to keep the guardrail function stateless

## Testing
- Python syntax verified for all 4 backend files
