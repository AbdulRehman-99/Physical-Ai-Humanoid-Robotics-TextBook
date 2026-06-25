---
id: 1
title: extract-intelligence-object
stage: misc
date: 2026-06-26
surface: agent
model: deepseek-v4-flash-free
feature: none
branch: main
user: muhib
command: write intelligence object file
labels: ["reverse-engineering", "intelligence-object", "patterns", "reusable-knowledge", "documentation"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - docs/reverse-engineered/intelligence-object.md
tests:
  - none
---

## Prompt

Write the file `D:\Abdul Rehman\GIAIC\Hackathon 1\Physical-Ai-Humanoid-Robotics-TextBook\docs\reverse-engineered\intelligence-object.md`

This is the reusable intelligence extracted from the "Physical AI & Humanoid Robotics Interactive Textbook" codebase. Follow the template from the SP reverse-engineering command.

Focus on extracting patterns that are reusable across projects, not just specific to this codebase.

Here are the patterns and skills to extract:

### Skill 1: RAG API with SSE Streaming (Backend)
**Pattern**: Build a streaming chat API using FastAPI + OpenAI Agents SDK + SSE
**Key files**: Backend/Connection/api.py, agent_wrapper.py, guardrails.py
**Persona**: Backend engineer building LLM-powered chat APIs with real-time streaming
**Principles**:
1. Use OpenAIChatCompletionsModel (not Responses API) when using OpenRouter — OpenRouter ignores instructions on /v1/responses
2. max_tokens limits output length and reduces latency; 150 tokens ≈ 80-100 words
3. Guardrails before LLM call: use @input_guardrail for off-topic detection with small max_tokens=10
4. Manual guardrail in streaming path: SDK guardrail events are unreliable in streamed mode
5. SSE for streaming: newline-delimited JSON, final event signals done with session_id
6. Session memory maintained server-side: last 5 turns, in-memory dict, TTL cleanup background task
7. Retry with exponential backoff (3 attempts, 2^attempt) for LLM API calls

### Skill 2: Context Resolution Strategy
**Pattern**: Resolve which context to use for RAG — user selection vs vector search
**Key files**: Backend/Connection/context_switcher.py, Backend/Agent/agent.py
**Persona**: Backend engineer implementing RAG context resolution
**Principles**:
1. Always prefer explicit user context (selected_text) over automatic retrieval
2. If no user context, fall back to vector search with configurable TOP_K
3. Enable fallback to LLM-only mode when vector DB is unavailable
4. Normalize chunk access patterns: handle dict vs object responses, check multiple content fields
5. Truncate context to ~6000 chars to fit LLM context windows

### Skill 3: Session Memory for Chat Applications
**Pattern**: Maintain conversation history per user session in-memory
**Key files**: Backend/Connection/api.py, Backend/Agent/main.py
**Persona**: Backend engineer adding conversation memory to chat APIs
**Principles**:
1. Use UUID v4 for session IDs (no sequential IDs)
2. Cap memory at N turns (typically 5) — trim oldest first
3. Implement TTL (1800s = 30 min) with background cleanup every 5 minutes
4. Pass memory as list of {"user": "...", "assistant": "..."} dicts
5. Build input items as alternating user/assistant messages for the LLM
6. Update memory AFTER successful response (not before)
7. Session is created on first request if no valid session_id provided

### Skill 4: Multi-Session Chat History with localStorage
**Pattern**: Persist multiple chat sessions in browser with auto-expiry
**Key files**: Frontend/src/components/ChatKit/ChatKit.tsx
**Persona**: Frontend developer adding persistent chat history to a React app
**Principles**:
1. Use 3 localStorage keys: active session ID, per-session messages, order array
2. Cap messages per session (100 max) to prevent quota issues
3. Auto-delete sessions older than 48h on any touchpoint (mount, save, list, etc.)
4. SSR-safe: wrap all localStorage calls in try/catch (Docusaurus builds in Node.js)
5. Debounce saves (300ms) to avoid excessive writes
6. Show session preview (first 60 chars of last message) + relative timestamp
7. Confirm before deleting a session

### Skill 5: Plain-Text Formatting for LLM Output
**Pattern**: Format LLM responses as plain text with inline labels, not markdown
**Key files**: Backend/Connection/agent_wrapper.py, Backend/Agent/agent.py
**Persona**: Developer writing LLM instruction prompts
**Principles**:
1. Use concrete examples in instructions — abstract rules are ignored by models
2. Format: "Heading:" on its own line, then bullets with "- Label: Description"
3. Explicitly ban markdown symbols (#, ##, **, backticks)
4. Target 80-100 words via max_tokens=150
5. Include the actual book context in the instructions (not abstract instructions about how to use context)

### Skill 6: Off-Topic Detection with LLM Guardrails
**Pattern**: Classify whether a user query is off-topic before processing it
**Key files**: Backend/Connection/guardrails.py, Backend/Agent/agent.py
**Persona**: Developer adding content safety to LLM-powered apps
**Principles**:
1. Use a small LLM call (max_tokens=10, temperature=0.1) for classification
2. System prompt: "Answer only 'yes' or 'no': Can this question be answered using the provided content?"
3. Provide book content as context (truncated to 1500 chars)
4. Reuse the same OpenRouter client and model for efficiency
5. Two-tier approach: SDK @input_guardrail for synchronous endpoints, manual check for streaming
6. Off-topic flag is sentinel-based: return special string "__OFF_TOPIC__" that caller checks
7. For streaming: first yielded token is the sentinel, then generator returns immediately

### Skill 7: sys.path Import Hack for Local Packages
**Pattern**: Import local packages that aren't installed as pip packages
**Key files**: Backend/Connection/context_switcher.py, Backend/Agent/agent.py
**Persona**: Developer working in a monorepo where sub-packages aren't installed
**Principles**:
1. Use sys.path.insert(0, str(path)) to add the package directory
2. Use relative paths from __file__ (Path(__file__).parent.parent / "Retrieval" / "src")
3. Must happen before any import from the package
4. Fragile — breaks if files are moved
5. Better approach: install as editable package (pip install -e) but sys.path hack is faster

### Architectural Decisions Inferred

#### ADR-001: OpenAIChatCompletionsModel over Responses API
**Status**: Accepted
**Context**: OpenRouter provider does not support instructions and max_output_tokens on the /v1/responses endpoint for Qwen models
**Decision**: Use OpenAIChatCompletionsModel from the Agents SDK
**Rationale**: ChatCompletions API fully supports system instructions and max_tokens parameters on OpenRouter
**Consequences**: Cannot use Responses API features (structured outputs, built-in tools); must configure model explicitly

#### ADR-002: In-Memory Sessions over Database
**Status**: Accepted
**Context**: No persistent storage requirement, single-server deployment
**Decision**: Store sessions in Python dict with TTL cleanup
**Consequences**: Sessions lost on server restart; not horizontally scalable; simple and fast for single-server

#### ADR-003: SSE over WebSocket
**Status**: Accepted
**Context**: Need real-time token streaming from LLM to frontend
**Decision**: Server-Sent Events (SSE) via FastAPI StreamingResponse
**Consequences**: Simpler than WebSocket (no bidirectional communication needed); works over HTTP/1.1; frontend uses ReadableStream API

#### ADR-004: Frontend localStorage over Server Storage for Chat History
**Status**: Accepted
**Context**: Chat history should persist across sessions without server-side storage
**Decision**: Store multi-session chat history in browser localStorage
**Consequences**: 5MB limit per origin; data tied to specific browser; no server-side backup; user can clear browser data

### Code Conventions

#### Pattern: Agent Wrapper / Adapter Pattern
Backend/Agent/adapter.py wraps OpenRouter client behind create_agent, run_agent, run_agent_streamed, chat_completions_create. Singleton instance at module level.

#### Pattern: Instruction Builder
Both agent_wrapper.py (Connection) and agent.py (Agent) build instructions using string interpolation with the actual context embedded. Three templates: selected_text, retrieved_chunks, no_context.

#### Pattern: SSE Streaming Generator
Both Connection and Agent use the same pattern: async generator yielding json.dumps({"token": token}) + "\n" per event, with {"done": true, "session_id": "..."} as final event.

Write this to `D:\Abdul Rehman\GIAIC\Hackathon 1\Physical-Ai-Humanoid-Robotics-TextBook\docs\reverse-engineered\intelligence-object.md`

Start with:
```
# Physical AI & Humanoid Robotics — Interactive Textbook Reusable Intelligence

**Version**: 1.0 (Extracted from Codebase)
**Date**: 2026-06-26
```

Fill ALL template sections. Do NOT truncate.

## Response snapshot

Wrote `docs/reverse-engineered/intelligence-object.md` (~600 lines) containing 7 reusable skills, 4 inferred ADRs, 3 code conventions, cross-cutting concerns, edge cases table, anti-patterns list, and a skill dependency map. Used the SP reverse-engineering command format with full template sections (Table of Contents, Skills with Persona/Principles/Usage instructions/Acceptance criteria, ADRs with Options Considered tables, Code Conventions, Cross-Cutting Concerns, Edge Cases, Anti-Patterns, Dependency Map).

## Outcome

- ✅ Impact: Extracted 7 reusable patterns from the codebase into a standalone intelligence object file under docs/reverse-engineered/
- 🧪 Tests: none
- 📁 Files: docs/reverse-engineered/intelligence-object.md (created)
- 🔁 Next prompts: review intelligence-object.md, use patterns in new projects
- 🧠 Reflection: The intelligence object format effectively distills project-specific implementations into cross-project reusable knowledge by separating the "what" (principles) from the "how" (implementation)

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS — file written successfully, all template sections filled, no truncation
- Prompt variant (if applicable): Standard SP reverse-engineering command
- Next experiment (smallest change to try): None
