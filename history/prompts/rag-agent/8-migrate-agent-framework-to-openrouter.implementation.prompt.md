---
id: 8
title: Migrate Agent Framework to OpenRouter
stage: refactor
date: 2026-06-25
surface: agent
model: deepseek-v4-flash-free
feature: rag-agent
branch: update-agent-framework
user: user
command: custom
labels: [migration,litellm,openrouter,openai-agent-sdk,refactor]
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
 - Backend/Agent/settings.py
 - Backend/Connection/agent_wrapper.py
 - Backend/requirements.txt
 - Backend/pyproject.toml
 - .env
 - uv.lock
tests: []
---

## Prompt

Requirement: Migrate the agent framework from LiteLLM to OpenAI Agent SDK.
API Configuration: Integrate OpenRouter as the provider (using OpenRouter API key) within the new OpenAI Agent SDK setup.
Constraint: Maintain the existing codebase structure as much as possible; minimize architectural changes outside of the agent's initialization and interaction logic.

## Response snapshot

Migrated both agent implementations from their previous frameworks to **OpenAI Agent SDK v0.17.5 + OpenRouter (Qwen 2.5 72B)**.

### Changes Made

| File | Before | After |
|------|--------|-------|
| `Backend/Agent/adapter.py` | `GeminiAdapter` wrapping `litellm.completion()` | `OpenRouterAdapter` using `AsyncOpenAI` + `Agent`/`Runner` from `openai-agents` SDK |
| `Backend/Agent/agent.py` | `self.gemini_client.chat_completions_create()` | `self.adapter.create_agent()` + `self.adapter.run_agent()` for main flow; `self.adapter.chat_completions_create()` for off-topic detection |
| `Backend/Agent/settings.py` | Gemini-only settings | Added `openrouter_api_key`, `openrouter_base_url`, `openrouter_model` |
| `Backend/Connection/agent_wrapper.py` | `google.generativeai` (native Gemini SDK) | `AsyncOpenAI` + `Agent`/`Runner` from `openai-agents` SDK |
| `Backend/requirements.txt` | Included `litellm`, `google-generativeai` | Removed both |
| `Backend/pyproject.toml` | Included `litellm`, `google-generativeai` | Removed both |
| `.env` | No OpenRouter config | Added `OPENROUTER_API_KEY`, `OPENROUTER_BASE_URL`, `OPENROUTER_MODEL` |

### Key Design Decisions

1. **Shared pattern across both services**: Both use the same approach — `AsyncOpenAI` client configured to OpenRouter's base URL, then `set_default_openai_client()` so `Runner.run()` works automatically.
2. **Dual interface in adapter**: `OpenRouterAdapter` provides both `create_agent()`/`run_agent()` (for main RAG flow via Agent SDK) and `chat_completions_create()` (for off-topic classification via raw API call with retry logic).
3. **Tracing disabled** via `set_tracing_disabled(True)` to prevent OpenRouter API key from being sent to OpenAI's trace endpoint.
4. **Bare model name**: `qwen-2.5-72b-instruct` (no prefix) so the Agent SDK's `MultiProvider` routes it to the OpenAI provider with our custom OpenRouter client.

### Issues Resolved During Testing

| Issue | Root Cause | Fix |
|-------|-----------|-----|
| Model slug `openrouter/qwen/qwen-2.5-72b-instruct` invalid | Wrong prefix — OpenRouter model IDs omit `openrouter/` | Queried OpenRouter `/api/v1/models` for correct slug |
| `set_tracing_disabled()` TypeError | Function requires `disabled: bool` argument | Changed to `set_tracing_disabled(True)` |
| `Unknown prefix: qwen` error | Agent SDK `MultiProvider` doesn't recognize `qwen/` prefix | Removed prefix — bare model name routes to OpenAI provider + custom client |

### Verification

- ✅ Raw chat completion via `chat_completions_create()` → `"yes"`
- ✅ Agent SDK `Runner.run()` via `run_agent()` → `"Hello."`
- ✅ 29 packages removed (`litellm`, `google-generativeai`, + 27 transitive)
- ✅ All imports verified
- ✅ Tracing disabled to protect API key

## Outcome

Full migration complete. Both `Backend/Agent/` (port 8001) and `Backend/Connection/` (port 8000) now use OpenAI Agent SDK v0.17.5 with OpenRouter as the provider (Qwen 2.5 72B). LiteLLM and google-generativeai dependencies removed. API connection tested and working.
