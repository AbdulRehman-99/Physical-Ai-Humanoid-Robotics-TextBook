---
id: 11
title: Increase Output Length and Add Formatting Instructions
stage: refactor
date: 2026-06-25
surface: agent
model: deepseek-v4-flash-free
feature: rag-agent
branch: main
user: user
command: custom
labels: [output-length,formatting,instructions]
links:
  spec: null
  plan: null
  tasks: null
  ticket: null
  adr: null
  pr: null
files:
 - Backend/Agent/adapter.py
 - Backend/Agent/agent.py
 - Backend/Connection/agent_wrapper.py
tests:
 - Backend/Ingestion/tests/test_chunker.py
---

# Increase Output Length and Add Formatting Instructions

## Summary
Increased the `max_tokens` limit from 300 to 1200 across both services, updated all instruction strings to remove the "3-4 sentence" constraint in favor of "500-800 words with headings/sub-headings/bullet points", and doubled the context window from 2000 to 4000 characters to support the longer answers.

## Changes

### Backend/Agent/adapter.py
- `max_tokens: int = 300` → `max_tokens: int = 1200` (default parameter in `create_agent()`)

### Backend/Agent/agent.py
- `max_tokens=300` → `max_tokens=1200` in both `process_message_streamed()` and `generate_response()` calls
- Both instruction blocks (selected_text and vector_retrieval): rule #4 changed from "Keep your answer concise — no more than 3-4 sentences" to "Write a thorough answer (500-800 words). Use headings, sub-headings, and bullet points to structure the response and make it engaging."

### Backend/Connection/agent_wrapper.py
- `ModelSettings(max_tokens=300)` → `ModelSettings(max_tokens=1200)` in both `_call_agent_streamed()` and `_call_agent_internal()`
- All three instruction blocks: replaced "Keep your answer concise — no more than 3-4 sentences." with "Write a thorough answer (500-800 words). Use headings, sub-headings, and bullet points to structure the response and make it engaging."
- `combined_context[:2000]` → `combined_context[:4000]` to provide more source material for longer answers

## Rationale
- 1200 tokens comfortably covers 500-800 words (avg 1.3 tokens/word)
- The explicit formatting instruction (headings, sub-headings, bullet points) was added per user request to make answers more interesting
- Context window doubled so the LLM has enough source material to produce the longer, structured responses

## Testing
- 8/8 chunker tests pass
- Agent module imports verified
