---
id: 13
title: Cap Tokens at 800 and Drop Markdown Formatting
stage: refactor
date: 2026-06-25
surface: agent
model: deepseek-v4-flash-free
feature: rag-agent
branch: main
user: user
command: custom
labels: [token-limit,formatting,plain-text]
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
 - Backend/Agent/agent.py
tests:
 - Backend/Ingestion/tests/test_chunker.py
---

# Cap Tokens at 800 and Drop Markdown Formatting

## Summary
Reduced `max_tokens` from 1200 to 800 across both services, changed instruction wording from "500-800 words" to "at most 800 words", and replaced markdown formatting instructions with plain text formatting.

## Changes

### max_tokens
- `agent_wrapper.py`: both `ModelSettings(max_tokens=1200)` → `ModelSettings(max_tokens=800)` (×2)
- `adapter.py`: default `max_tokens: int = 1200` → `max_tokens: int = 800`
- `agent.py`: both `max_tokens=1200` → `max_tokens=800` (×2)

### Instructions (all 5 blocks)
- `"500-800 words"` → `"at most 800 words"`
- `"Use headings, sub-headings, and bullet points to structure the response"` → `"Use plain text headings, sub-headings, and bullet points. Do NOT use markdown formatting (no ##, no **, no backticks)"`

## Testing
- Agent module imports verified
