---
id: 12
title: Force ChatCompletions API for Instructions and Token Limit
stage: refactor
date: 2026-06-25
surface: agent
model: deepseek-v4-flash-free
feature: rag-agent
branch: main
user: user
command: custom
labels: [bugfix,responses-api,chatcompletions,instructions,max-tokens]
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

# Force ChatCompletions API to Fix Instructions and Token Limit

## Root Cause
The OpenAI Agent SDK v0.17.7 uses the **Responses API** (`POST /v1/responses`) by default when an `AsyncOpenAI` client is configured. The Responses API sends:
- `instructions` as a top-level field (not a system message)
- `max_tokens` as `max_output_tokens`

OpenRouter's `/v1/responses` implementation for Qwen models ignores both `instructions` and `max_output_tokens`, so the model never received the system prompt instructing it to write 500-800 words with formatting, and the output token limit was not enforced.

## Fix
Forced both services to use the **Chat Completions API** (`POST /v1/chat/completions`) by explicitly instantiating `OpenAIChatCompletionsModel(model=model_name, openai_client=client)` and passing the model *object* to `Agent(model=model_obj)` instead of a string.

The Chat Completions API sends:
- System prompt as `messages[0] = {"role": "system", "content": "..."}` → fully supported by OpenRouter
- `max_tokens` as a standard parameter → fully supported by OpenRouter

## Changes

### Backend/Connection/agent_wrapper.py
- Added import: `from agents.models.openai_chatcompletions import OpenAIChatCompletionsModel`
- `_call_agent_streamed()`: created `OpenAIChatCompletionsModel` with `self.client`, passed to `Agent(model=model_obj)`
- `_call_agent_internal()`: same pattern
- `_build_instructions()`: all 3 branches — "Write a thorough answer (500-800 words)" → "You MUST write a thorough answer of 500-800 words. You MUST use headings, sub-headings, and bullet points"; context window `[:4000]` → `[:6000]`

### Backend/Agent/adapter.py
- Added import: `from agents.models.openai_chatcompletions import OpenAIChatCompletionsModel`
- `create_agent()`: creates `OpenAIChatCompletionsModel` using `self.client` and passes `model=model_obj` to `Agent()`

### Backend/Agent/agent.py
- Both instruction blocks: "Write a thorough answer (500-800 words)" → "You MUST write a thorough answer of 500-800 words. You MUST use headings, sub-headings, and bullet points"

## Testing
- 8/8 chunker tests pass
- Agent module imports verified
- Connection module syntax verified
