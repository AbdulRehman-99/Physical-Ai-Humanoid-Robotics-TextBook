---
id: 17
title: Reduce Max Tokens to 150 for 80-100 Word Answers
stage: refactor
date: 2026-06-25
surface: agent
model: deepseek-v4-flash-free
feature: rag-agent
branch: main
user: user
command: custom
labels: [max-tokens,length,conciseness,80-100-words]
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
 - (none)
---

# Reduce Max Tokens to 150 for 80-100 Word Answers

## Summary
Reduced max_tokens from 400 to 150 across all agent calls to target 80-100 word answers. Updated instruction text from "Keep the whole answer concise with maximum 3 sections" to "Keep the answer very short (80-100 words, max 2 sections)".

## Changes

### max_tokens: 400 → 150 (5 locations)
- `Backend/Agent/adapter.py:65` — default parameter
- `Backend/Agent/agent.py:215` — process_message_streamed call
- `Backend/Agent/agent.py:238` — generate_response call
- `Backend/Connection/agent_wrapper.py:73` — streamed call
- `Backend/Connection/agent_wrapper.py:226` — non-streamed call

### Instruction text: 5 blocks
- `Backend/Agent/agent.py:73,101` — "maximum 3 sections" → "very short (80-100 words, max 2 sections)"
- `Backend/Connection/agent_wrapper.py:141,175,200` — same change

## Testing
- Python syntax verified for all 3 files
