# Physical AI & Humanoid Robotics — Interactive Textbook Reusable Intelligence

**Version**: 1.0 (Extracted from Codebase)
**Date**: 2026-06-26

---

## Table of Contents

1. [Skills (Reusable Patterns)](#1-skills-reusable-patterns)
   - [Skill 1: RAG API with SSE Streaming](#skill-1-rag-api-with-sse-streaming-backend)
   - [Skill 2: Context Resolution Strategy](#skill-2-context-resolution-strategy)
   - [Skill 3: Session Memory for Chat Applications](#skill-3-session-memory-for-chat-applications)
   - [Skill 4: Multi-Session Chat History with localStorage](#skill-4-multi-session-chat-history-with-localstorage)
   - [Skill 5: Plain-Text Formatting for LLM Output](#skill-5-plain-text-formatting-for-llm-output)
   - [Skill 6: Off-Topic Detection with LLM Guardrails](#skill-6-off-topic-detection-with-llm-guardrails)
   - [Skill 7: sys.path Import Hack for Local Packages](#skill-7-syspath-import-hack-for-local-packages)
2. [Architectural Decisions Inferred](#2-architectural-decisions-inferred)
   - [ADR-001: OpenAIChatCompletionsModel over Responses API](#adr-001-openaichatcompletionsmodel-over-responses-api)
   - [ADR-002: In-Memory Sessions over Database](#adr-002-in-memory-sessions-over-database)
   - [ADR-003: SSE over WebSocket](#adr-003-sse-over-websocket)
   - [ADR-004: Frontend localStorage over Server Storage for Chat History](#adr-004-frontend-localstorage-over-server-storage-for-chat-history)
3. [Code Conventions](#3-code-conventions)
   - [Pattern: Agent Wrapper / Adapter Pattern](#pattern-agent-wrapper--adapter-pattern)
   - [Pattern: Instruction Builder](#pattern-instruction-builder)
   - [Pattern: SSE Streaming Generator](#pattern-sse-streaming-generator)

---

## 1. Skills (Reusable Patterns)

### Skill 1: RAG API with SSE Streaming (Backend)

**Pattern**: Build a streaming chat API using FastAPI + OpenAI Agents SDK + SSE

**Key files**: `Backend/Connection/api.py`, `agent_wrapper.py`, `guardrails.py`

**Persona**: Backend engineer building LLM-powered chat APIs with real-time streaming

**Principles**:

1. **OpenAIChatCompletionsModel, not Responses API** — OpenRouter ignores `instructions` and `max_output_tokens` on the `/v1/responses` endpoint. Always use `OpenAIChatCompletionsModel` from the Agents SDK when routing through OpenRouter.

2. **max_tokens=150** — Limits output to approximately 80–100 words, reducing latency and keeping responses concise. Applied consistently across all LLM calls in the service.

3. **Guardrails before LLM call** — Use the SDK's `@input_guardrail` decorator for synchronous endpoints. The guardrail itself uses a small LLM call with `max_tokens=10` and `temperature=0.1` to classify off-topic queries before the main LLM call.

4. **Manual guardrail in streaming path** — SDK guardrail events are unreliable in streamed mode (`run_streamed()`). Instead, perform the guardrail check manually before starting the stream. The first yielded token is a sentinel (`__OFF_TOPIC__`) that the caller checks, after which the generator returns immediately.

5. **SSE for streaming** — Use `StreamingResponse` from FastAPI with a `text/event-stream` media type. Each event is a newline-delimited JSON line: `json.dumps({"token": token}) + "\n"`. The final event signals completion: `json.dumps({"done": true, "session_id": "..."}) + "\n"`.

6. **Session memory maintained server-side** — Last 5 turns stored in an in-memory `Dict[str, List[Dict]]` with UUID keys. Background cleanup task (every 5 minutes) removes sessions past TTL (1800 seconds / 30 minutes).

7. **Retry with exponential backoff** — 3 attempts with `2^attempt` seconds sleep between retries. Applied around the OpenRouter API call to handle transient failures.

**Usage instructions**:

```python
# Example: FastAPI streaming endpoint
from fastapi.responses import StreamingResponse
from openai import AsyncOpenAI
from agents import Agent, Runner, OpenAIChatCompletionsModel, ModelSettings

model = OpenAIChatCompletionsModel(
    model="qwen-2.5-72b-instruct",
    openai_client=AsyncOpenAI(
        api_key=settings.openrouter_api_key,
        base_url=settings.openrouter_base_url,
    )
)
agent = Agent(
    name="Assistant",
    instructions="...",
    model=model,
    model_settings=ModelSettings(max_tokens=150),
)

async def event_generator():
    result = Runner.run_streamed(agent, input=prompt)
    async for event in result.stream_events():
        if event.type == "raw_response_event" and event.data.type == "response.output_text.delta":
            yield json.dumps({"token": event.data.delta}) + "\n"
    yield json.dumps({"done": True, "session_id": session_id}) + "\n"

return StreamingResponse(event_generator(), media_type="text/event-stream")
```

**Acceptance criteria**:

- [ ] SSE stream delivers tokens as newline-delimited JSON
- [ ] Final event contains `{"done": true, "session_id": "..."}`
- [ ] Off-topic queries return `__OFF_TOPIC__` as first token (streaming) or blocked by guardrail (sync)
- [ ] Retry logic handles transient failures (test with invalid API key then valid key)
- [ ] All synchronous endpoints have `@input_guardrail` applied

---

### Skill 2: Context Resolution Strategy

**Pattern**: Resolve which context to use for RAG — user selection vs vector search

**Key files**: `Backend/Connection/context_switcher.py`, `Backend/Agent/agent.py`

**Persona**: Backend engineer implementing RAG context resolution

**Principles**:

1. **Prefer explicit user context** — If the user has selected text (`selected_text`), use it directly as the context for LLM instructions. This gives the user explicit control over what the assistant refers to.

2. **Fall back to vector search** — If no `selected_text` is provided, perform a vector search against Qdrant using the user's query as the search term. Configurable `TOP_K` (default 3) controls how many chunks are returned.

3. **LLM-only fallback** — When the vector database is unavailable (connection error, timeout), fall back to LLM-only mode with a system prompt that acknowledges the lack of book context. Do not crash.

4. **Normalize chunk access** — Vector search results may come as dicts or objects. Check for multiple content fields (`content`, `text`, `page_content`) and normalize to a single string. Handle both list and single-item returns.

5. **Truncate context** — Cap context at approximately 6000 characters to fit within LLM context windows. Use Python string slicing: `context[:6000]`.

**Usage instructions**:

```python
# Example: Context resolution logic
async def resolve_context(query: str, selected_text: str = None) -> tuple[str, str | None]:
    if selected_text:
        return selected_text[:6000], "selected_text"

    try:
        chunks = await search_similar(query, top_k=TOP_K)
        context = normalize_chunks(chunks)
        return context[:6000], "retrieved"
    except QdrantException:
        return None, "no_context"  # LLM-only fallback
```

**Acceptance criteria**:

- [ ] `selected_text` takes priority over vector search when provided
- [ ] Vector search fallback works with configurable `TOP_K`
- [ ] Chunk normalization handles both dict and object response types
- [ ] Context is truncated to ~6000 characters
- [ ] Vector DB failure falls back gracefully with LLM-only mode

---

### Skill 3: Session Memory for Chat Applications

**Pattern**: Maintain conversation history per user session in-memory

**Key files**: `Backend/Connection/api.py`, `Backend/Agent/main.py`

**Persona**: Backend engineer adding conversation memory to chat APIs

**Principles**:

1. **UUID v4 for session IDs** — Use `uuid.uuid4()` to generate session identifiers. No sequential IDs to prevent session enumeration.

2. **Cap memory at N turns** — Store a maximum of 5 conversation turns (user + assistant pairs). When a new turn exceeds the limit, remove the oldest turn first (FIFO eviction).

3. **Implement TTL with background cleanup** — Session Time-to-Live is 1800 seconds (30 minutes). A background `asyncio.create_task` runs every 5 minutes, removing sessions whose idle time exceeds TTL. Each session stores its `last_accessed` timestamp.

4. **Pass memory as dict list** — Format session history as `[{"user": "...", "assistant": "..."}, ...]`. Build alternating user/assistant messages from this list when constructing the LLM input.

5. **Update memory AFTER successful response** — Append the new user/assistant pair to the session only after the LLM call succeeds. This prevents storing failed or partial responses.

6. **Session creation on first request** — If no `session_id` is provided or the provided ID is not found in the session store, create a new session with a fresh UUID and empty history.

**Usage instructions**:

```python
# Example: Session memory management
import uuid
from datetime import datetime, timedelta

sessions: dict[str, dict] = {}
SESSION_TTL = 1800  # 30 minutes
MAX_TURNS = 5

async def get_or_create_session(session_id: str | None) -> tuple[str, list]:
    if session_id and session_id in sessions:
        session = sessions[session_id]
        session["last_accessed"] = datetime.now()
        return session_id, session["history"]

    new_id = str(uuid.uuid4())
    sessions[new_id] = {"history": [], "created": datetime.now(), "last_accessed": datetime.now()}
    return new_id, []

async def update_session(session_id: str, user_msg: str, assistant_msg: str) -> None:
    session = sessions[session_id]
    session["history"].append({"user": user_msg, "assistant": assistant_msg})
    if len(session["history"]) > MAX_TURNS:
        session["history"] = session["history"][-MAX_TURNS:]
    session["last_accessed"] = datetime.now()

async def cleanup_sessions():
    while True:
        now = datetime.now()
        expired = [sid for sid, s in sessions.items()
                   if (now - s["last_accessed"]).total_seconds() > SESSION_TTL]
        for sid in expired:
            del sessions[sid]
        await asyncio.sleep(300)  # every 5 minutes
```

**Acceptance criteria**:

- [ ] New session created when no `session_id` is provided
- [ ] Session history capped at 5 turns (oldest removed first)
- [ ] History is only updated after successful LLM response
- [ ] Sessions expired after 30 minutes of inactivity
- [ ] Background cleanup task removes expired sessions every 5 minutes

---

### Skill 4: Multi-Session Chat History with localStorage

**Pattern**: Persist multiple chat sessions in browser with auto-expiry

**Key file**: `Frontend/src/components/ChatKit/ChatKit.tsx`

**Persona**: Frontend developer adding persistent chat history to a React app

**Principles**:

1. **Three localStorage keys** — Use a structured storage approach:
   - `activeSessionId`: UUID of the currently active session
   - `sessions`: JSON object mapping session IDs to their message arrays
   - `sessionOrder`: JSON array of session IDs in display order

2. **Cap messages per session** — Maximum 100 messages per session to prevent exceeding the 5MB localStorage quota. When a new message exceeds the limit, remove the oldest message.

3. **Auto-delete old sessions** — On any touchpoint (mount, save, list, delete), scan all sessions and remove those older than 48 hours. Check timestamps stored with each session.

4. **SSR-safe** — All `localStorage` and `window` calls must be wrapped in `try/catch` blocks. Docusaurus builds in Node.js where these APIs are unavailable. Use a helper function: `const safeGetItem = (key: string): string | null => { try { return localStorage.getItem(key); } catch { return null; } };`

5. **Debounce saves** — Use a 300ms debounce on `localStorage.setItem` calls to avoid excessive writes during rapid interactions (e.g., streaming tokens being appended).

6. **Session preview UI** — Display each session in a sidebar with:
   - First 60 characters of the last message as the session title
   - Relative timestamp (e.g., "2m ago", "1h ago", "3d ago")
   - A delete button with confirmation dialog before removal

7. **Confirm before deleting** — Use `window.confirm("Delete this session?")` before permanently removing a session from localStorage. On confirmation, remove from all three keys and update the UI.

**Usage instructions**:

```typescript
// Example: SSR-safe localStorage helper
const safeLocalStorage = {
  getItem(key: string): string | null {
    try { return localStorage.getItem(key); } catch { return null; }
  },
  setItem(key: string, value: string): void {
    try { localStorage.setItem(key, value); } catch { /* quota exceeded */ }
  },
  removeItem(key: string): void {
    try { localStorage.removeItem(key); } catch { /* noop */ }
  }
};

// Example: Session expiry check
const SESSION_MAX_AGE_MS = 48 * 60 * 60 * 1000; // 48 hours
function isSessionExpired(timestamp: number): boolean {
  return Date.now() - timestamp > SESSION_MAX_AGE_MS;
}

// Example: Debounced save
const debouncedSave = useMemo(
  () => debounce((sessions: Record<string, Session>) => {
    safeLocalStorage.setItem('sessions', JSON.stringify(sessions));
  }, 300),
  []
);
```

**Acceptance criteria**:

- [ ] Multiple chat sessions are persisted across page refreshes
- [ ] Sessions older than 48 hours are auto-deleted on any touchpoint
- [ ] localStorage calls are SSR-safe (no errors during build)
- [ ] Saves are debounced at 300ms
- [ ] Session preview shows first 60 chars + relative timestamp
- [ ] Delete requires user confirmation before removal
- [ ] Maximum 100 messages per session enforced

---

### Skill 5: Plain-Text Formatting for LLM Output

**Pattern**: Format LLM responses as plain text with inline labels, not markdown

**Key files**: `Backend/Connection/agent_wrapper.py`, `Backend/Agent/agent.py`

**Persona**: Developer writing LLM instruction prompts

**Principles**:

1. **Concrete examples in instructions** — Abstract formatting rules are frequently ignored by LLMs. Provide explicit, concrete examples of the desired output format directly in the system prompt.

2. **Colon-label format** — Use `"Heading:"` on its own line, followed by bullet points with `"- Label: Description"`. This gives structure without markdown syntax.

3. **Explicitly ban markdown** — Tell the model: "Do NOT use any markdown symbols (#, ##, **, *, backticks, etc.)." Include this prohibition near the end of the instructions.

4. **Target 80–100 words via max_tokens=150** — Token count is approximately 1.5x word count for English. 150 tokens yields about 80–100 words, keeping responses brief and scannable.

5. **Embed actual context in instructions** — Rather than giving abstract instructions about how to use the book context ("use the following context to answer"), inject the actual context text into the instructions string. Models respond better to in-context examples.

**Usage instructions**:

```python
# Example: Instruction builder with concrete formatting
def build_instructions(context: str, history: list[dict]) -> str:
    return f"""You are a helpful textbook assistant. Answer questions about "Physical AI & Humanoid Robotics."

Use this book content to answer:
---BOOK CONTENT---
{context}
---END BOOK CONTENT---

Rules:
- Answer in 80-100 words
- Use ONLY plain text - NO markdown symbols (#, ##, **, backticks)
- Format like this:

Key Points:
- Topic: Brief explanation
- Example: Specific illustration
- Note: Additional detail

Conversation history:
{format_history(history)}"""
```

**Acceptance criteria**:

- [ ] Responses contain no markdown symbols (`#`, `**`, `` ` ``, etc.)
- [ ] Responses use colon-label format for structure
- [ ] Responses are 80–100 words (verified by token count)
- [ ] The actual book context is embedded in the instructions string
- [ ] Concrete formatting examples are included in the prompt

---

### Skill 6: Off-Topic Detection with LLM Guardrails

**Pattern**: Classify whether a user query is off-topic before processing it

**Key files**: `Backend/Connection/guardrails.py`, `Backend/Agent/agent.py`

**Persona**: Developer adding content safety to LLM-powered apps

**Principles**:

1. **Small LLM call for classification** — Use a separate LLM call with `max_tokens=10` and `temperature=0.1` for classification. This is fast (single token output) and cheap compared to the main response.

2. **System prompt for binary classification** — Use a simple system prompt: `"Answer only 'yes' or 'no': Can this question be answered using the provided content?"` The model must output exactly "yes" or "no".

3. **Provide book content as context** — Pass the same retrieved context (truncated to ~1500 chars) as part of the guardrail prompt. The classification is grounded in the actual book content, not just general topic matching.

4. **Reuse the same OpenRouter client** — Use the same `AsyncOpenAI` client and model for the guardrail call. This avoids creating duplicate connections and keeps configuration centralized.

5. **Two-tier approach** — For synchronous endpoints, use the SDK's `@input_guardrail` decorator which automatically blocks the request. For streaming endpoints, perform a manual check before starting the stream.

6. **Sentinel-based flag** — The off-topic signal is a sentinel string `"__OFF_TOPIC__"` that callers check. In the streaming path, the first yielded token is the sentinel, then the generator returns immediately without streaming more tokens.

7. **Case-insensitive check** — Normalize the guardrail response: `response.strip().lower().startswith("no")` to handle variations like "No," or "no." or "No."

**Usage instructions**:

```python
# Example: Off-topic classification
async def check_off_topic(query: str, context: str) -> bool:
    response = await client.chat.completions.create(
        model=settings.model,
        messages=[
            {"role": "system", "content": "Answer only 'yes' or 'no': "
             "Can this question be answered using the provided content?"},
            {"role": "user", "content": f"Content:\n{context[:1500]}\n\nQuestion:\n{query}"}
        ],
        max_tokens=10,
        temperature=0.1,
    )
    answer = response.choices[0].message.content.strip().lower()
    return not answer.startswith("no")  # True = on-topic, False = off-topic

# Example: Manual guardrail for streaming
async def guarded_stream(query: str, context: str, session_id: str):
    is_on_topic = await check_off_topic(query, context)
    if not is_on_topic:
        yield json.dumps({"type": "off_topic", "content": "I can only answer questions about Physical AI and Humanoid Robotics."}) + "\n"
        return
    # ... proceed with streaming
```

**Acceptance criteria**:

- [ ] Guardrail classification uses `max_tokens=10` and `temperature=0.1`
- [ ] Context is truncated to ~1500 chars for guardrail prompt
- [ ] SDK `@input_guardrail` decorator applied to synchronous endpoints
- [ ] Manual guardrail check performed before streaming
- [ ] Off-topic queries return sentinel `__OFF_TOPIC__` (streaming) or 400 response (sync)
- [ ] Response normalization handles "No," / "no." / "No." variations

---

### Skill 7: sys.path Import Hack for Local Packages

**Pattern**: Import local packages that aren't installed as pip packages

**Key files**: `Backend/Connection/context_switcher.py`, `Backend/Agent/agent.py`

**Persona**: Developer working in a monorepo where sub-packages aren't installed

**Principles**:

1. **sys.path.insert(0, str(path))** — Insert the package directory at position 0 of `sys.path` so it takes priority over installed packages. Use `sys.path.insert(0, str(package_path))` not `sys.path.append`.

2. **Relative paths from __file__** — Compute paths relative to the current file using `Path(__file__).parent.parent / "Retrieval" / "src"`. This makes the hack work regardless of the absolute location of the workspace.

3. **Before any import** — The path manipulation must happen before any `import` or `from` statement that references the target package. Typically placed at the top of the file after standard library imports.

4. **Fragile** — The hack breaks if files are moved or the directory structure changes. It's invisible to tooling (linters, type checkers, IDEs) and can cause confusing import errors.

5. **Better approach** — Installing as an editable package (`pip install -e ../Retrieval/src`) is the proper solution. The `sys.path` hack is faster to set up but is technical debt.

**Usage instructions**:

```python
# Example: sys.path import hack
import sys
from pathlib import Path

# Add Retrieval module to path
_retrieval_path = Path(__file__).parent.parent / "Retrieval" / "src"
if str(_retrieval_path) not in sys.path:
    sys.path.insert(0, str(_retrieval_path))

# Now import from the Retrieval package
from retrieval import search_similar, get_book_context
```

**Acceptance criteria**:

- [ ] Package can be imported after the `sys.path` manipulation
- [ ] Path is computed relative to the current file
- [ ] Import works regardless of working directory
- [ ] `sys.path` is not polluted if the path is already present
- [ ] File moves (e.g., `Connection/` → `Services/Connection/`) will break the import

---

## 2. Architectural Decisions Inferred

### ADR-001: OpenAIChatCompletionsModel over Responses API

**Status**: Accepted

**Context**: OpenRouter provider does not support `instructions` and `max_output_tokens` on the `/v1/responses` endpoint for Qwen models. The Responses API is the newer OpenAI API, but OpenRouter's implementation is incomplete.

**Decision**: Use `OpenAIChatCompletionsModel` from the Agents SDK instead of the default Responses API model.

**Options considered**:

| Option | Trade-off |
|---|---|
| Responses API (default SDK) | Incompatible with OpenRouter — system instructions ignored |
| OpenAIChatCompletionsModel | Fully supports instructions and max_tokens on OpenRouter |
| Direct OpenAI SDK (no Agents SDK) | More control but loses Agent SDK features (guardrails, handoffs) |

**Rationale**: ChatCompletions API fully supports system instructions and `max_tokens` parameters on OpenRouter. The Agents SDK's `OpenAIChatCompletionsModel` adapter provides the best balance: full SDK feature support + OpenRouter compatibility.

**Consequences**:
- Cannot use Responses API features (structured outputs, built-in tools)
- Must explicitly configure model with `model=`, `openai_client=`, and `ModelSettings(max_tokens=...)`
- Locked into ChatCompletions API paradigm

**Affected files**:
- `Backend/Connection/agent_wrapper.py`
- `Backend/Agent/adapter.py`
- `Backend/Agent/agent.py`

---

### ADR-002: In-Memory Sessions over Database

**Status**: Accepted

**Context**: No persistent storage requirement for session data. Single-server deployment with no expectation of horizontal scaling. Sessions only need to survive the lifetime of API interactions (minutes to hours).

**Decision**: Store sessions in a Python `Dict[str, Dict]` with TTL cleanup via `asyncio.create_task`.

**Options considered**:

| Option | Trade-off |
|---|---|
| In-memory dict | Simple, fast; lost on restart, not scalable |
| Redis | Persistent, scalable; adds infrastructure dependency |
| SQLite | Persistent; adds file I/O, slower than memory |

**Rationale**: For a single-server textbook demo, an in-memory dict provides the simplest implementation with zero infrastructure. Sessions are ephemeral by nature (30-minute TTL), so persistence is not valuable. The background cleanup task prevents unbounded memory growth.

**Consequences**:
- Sessions lost on server restart (acceptable for demo/scenario)
- Not horizontally scalable (acceptable for single-server deployment)
- Simple and fast for single-server operation
- No infrastructure dependencies

**Affected files**:
- `Backend/Connection/api.py`
- `Backend/Agent/main.py`

---

### ADR-003: SSE over WebSocket

**Status**: Accepted

**Context**: Need real-time token streaming from the LLM to the frontend. Only server-to-client streaming is required — no bidirectional real-time communication.

**Decision**: Server-Sent Events (SSE) via FastAPI `StreamingResponse` with `media_type="text/event-stream"`.

**Options considered**:

| Option | Trade-off |
|---|---|
| SSE | Simple HTTP; works over HTTP/1.1; native browser API (EventSource/ReadableStream) |
| WebSocket | Bidirectional; more complex; requires upgrade handshake |
| Long polling | Works everywhere; inefficient; higher latency |

**Rationale**: SSE is the simplest protocol for unidirectional server-to-client streaming. It works over standard HTTP/1.1, requires no special proxy configuration, and can be consumed in the browser with `ReadableStream` (more flexible than `EventSource` for JSON parsing).

**Consequences**:
- No bidirectional communication (not needed — frontend sends requests via POST)
- Frontend uses `ReadableStream` API (not `EventSource`) for custom JSON parsing
- Works through standard HTTP proxies and load balancers
- Backend uses FastAPI `StreamingResponse` with async generator

**Affected files**:
- `Backend/Connection/api.py`
- `Backend/Agent/main.py`
- `Frontend/src/components/ChatKit/ChatKit.tsx`

---

### ADR-004: Frontend localStorage over Server Storage for Chat History

**Status**: Accepted

**Context**: Chat history should persist across browser sessions without requiring server-side storage. The user's conversation history is personal to their browser and does not need to be available across devices.

**Decision**: Store multi-session chat history in browser `localStorage`.

**Options considered**:

| Option | Trade-off |
|---|---|
| localStorage | 5MB limit per origin; no server cost; data tied to browser; user can clear |
| Server-side DB | Persistent across devices; adds storage cost; privacy considerations |
| IndexedDB | Larger storage (50MB+); more complex API; async |

**Rationale**: localStorage provides the simplest API for storing structured JSON data. The 5MB limit is sufficient for ~100 chat sessions at the configured message cap (100 messages/session). No server-side storage means no privacy concerns and zero infrastructure cost.

**Consequences**:
- 5MB limit per origin — enforced via 100-message cap per session
- Data tied to specific browser — sessions don't roam across devices
- No server-side backup — clearing browser data loses all history
- SSR complexity — all localStorage calls need `try/catch` for Node.js builds
- Sessions auto-delete after 48 hours to prevent stale data accumulation

**Affected files**:
- `Frontend/src/components/ChatKit/ChatKit.tsx`

---

## 3. Code Conventions

### Pattern: Agent Wrapper / Adapter Pattern

**Location**: `Backend/Agent/adapter.py`

**Purpose**: Abstract OpenRouter / OpenAI SDK behind a common interface.

**Structure**:

```python
class OpenRouterAdapter:
    def __init__(self):
        self.client = AsyncOpenAI(api_key=..., base_url=...)
        self._model = OpenAIChatCompletionsModel(model=..., openai_client=self.client)

    def create_agent(self, instructions: str) -> Agent: ...
    async def run_agent(self, agent: Agent, input: str) -> str: ...
    async def run_agent_streamed(self, agent: Agent, input: str): ...
    async def chat_completions_create(self, messages: list, max_tokens: int) -> str: ...

# Module-level singleton
agent_adapter = OpenRouterAdapter()
```

**Key behaviors**:
- Singleton instance at module level — no class instantiation per request
- Retry with exponential backoff on `chat_completions_create`
- Model configuration centralized in `__init__`
- Used by both Connection and Agent services

---

### Pattern: Instruction Builder

**Location**: `Backend/Connection/agent_wrapper.py`, `Backend/Agent/agent.py`

**Purpose**: Build LLM system instructions by embedding actual context, not abstract rules.

**Structure**:

```python
def build_instructions(context: str | None, history: list[dict], source: str) -> str:
    if source == "selected_text":
        template = INSTRUCTION_TEMPLATES["selected_text"]
    elif source == "retrieved":
        template = INSTRUCTION_TEMPLATES["retrieved"]
    else:
        template = INSTRUCTION_TEMPLATES["no_context"]

    return template.format(
        book_content=context or "No book content available.",
        history=format_history(history)
    )
```

**Three templates**:
- **selected_text** — User selected specific text; answer based on that selection
- **retrieved_chunks** — Retrieved from vector search; answer based on search results
- **no_context** — Neither available; acknowledge limitation and answer from general knowledge

**Key behaviors**:
- Context is embedded directly into instructions (not referenced abstractly)
- History formatted as alternating user/assistant messages
- Concrete formatting examples included

---

### Pattern: SSE Streaming Generator

**Location**: `Backend/Connection/api.py`, `Backend/Agent/main.py`

**Purpose**: Stream LLM tokens to the frontend as server-sent events.

**Structure**:

```python
async def generate_stream(prompt: str, session_id: str, context: str):
    # 1. Guardrail check
    is_on_topic = await check_off_topic(prompt, context)
    if not is_on_topic:
        yield json.dumps({"type": "off_topic", "content": "..."}) + "\n"
        return

    # 2. Build agent with instructions
    agent = adapter.create_agent(instructions=build_instructions(context, history, source))

    # 3. Stream tokens
    result = Runner.run_streamed(agent, input=prompt)
    async for event in result.stream_events():
        if event.type == "raw_response_event" and hasattr(event.data, "delta"):
            yield json.dumps({"token": event.data.delta}) + "\n"

    # 4. Final event
    yield json.dumps({"done": True, "session_id": session_id}) + "\n"
```

**Key behaviors**:
- Each token event: `{"token": "..."}\n`
- Off-topic event: `{"type": "off_topic", "content": "..."}\n`
- Completion event: `{"done": true, "session_id": "..."}\n`
- FastAPI returns `StreamingResponse(generator(), media_type="text/event-stream")`
- Frontend consumes via `ReadableStream` with custom line-by-line JSON parser

---

## 4. Cross-Cutting Concerns

### Error Handling

- **Guardrail failure**: Returns 400 with `{"detail": "Query is off-topic"}` (sync) or sentinel token `__OFF_TOPIC__` (streaming)
- **LLM API failure**: Retry 3 times with exponential backoff; if all fail, return 503
- **Vector DB failure**: Fall back to LLM-only mode (no crash)
- **Invalid input**: Returns 422 with FastAPI validation error
- **Rate limit exceeded**: Returns 429 with `Retry-After` header

### Configuration Management

- Environment variables loaded from `.env` at repo root
- Both services search multiple parent directories for `.env`
- `settings.py` (Agent) and `config.py` (Connection) centralize all configuration
- Module-level singleton pattern for settings instance

### Testing Strategy

- Agent has `pytest` test suite (`Backend/Agent/tests/`)
- Connection has no integration tests — relies on manual testing
- Frontend has no unit tests — relies on type checking (`npm run typecheck`)
- `error_handling.py` (Connection) is only imported by tests — dead production code

---

## 5. Edge Cases

| Edge Case | Handling |
|---|---|
| Empty user message | FastAPI validation rejects with 422 |
| Very long message (>500 chars) | FastAPI validation rejects with 422 |
| Vector DB timeout | Caught exception, fall back to LLM-only mode |
| OpenRouter API down | 3 retries with exponential backoff, then 503 |
| Off-topic query | Guardrail blocks before LLM call (sync) or sentinel (streaming) |
| Expired session | New session created automatically |
| No history in session | Empty history list, no context passed to LLM |
| localStorage quota exceeded | `try/catch` catches `QuotaExceededError`, write skipped |
| SSR (Node.js build) | All `localStorage` calls in `try/catch` return `null` |
| Concurrent requests to same session | Sequential processing per session ID |

---

## 6. Anti-Patterns to Avoid

- **Using Responses API with OpenRouter** — `instructions` and `max_output_tokens` are silently ignored
- **Updating session memory before response** — Failed/cancelled requests leave partial state in history
- **Markdown in LLM output** — Docusaurus renders it, but the textbook format is plain text with inline labels
- **Relying on SDK guardrails in streaming mode** — `@input_guardrail` events are unreliable with `run_streamed()`
- **Abstract formatting rules in system prompts** — Models follow concrete examples, not abstract instructions
- **Hardcoded paths** — Use `Path(__file__).parent` for relative paths, never hardcoded absolute paths
- **Synchronous localStorage access at module scope** — Breaks SSR; always wrap in `try/catch` inside lifecycle methods
- **Importing Retrieval without sys.path hack** — Will fail with `ModuleNotFoundError`

---

## 7. Skill Dependencies Map

```
Skill 1 (RAG API with SSE) ─┬─ Skill 2 (Context Resolution)
                              ├─ Skill 3 (Session Memory)
                              ├─ Skill 5 (Plain-Text Formatting)
                              └─ Skill 6 (Off-Topic Detection)

Skill 2 (Context Resolution) ─┬─ Skill 7 (sys.path Hack)
                               └─ External: Qdrant client

Skill 4 (localStorage History) ─┬─ Independent (frontend only)
                                 └─ Depends on: Skill 1 SSE format

Skill 7 (sys.path Hack) ─── Prerequisite for: Skill 2
```

---

*Extracted from the Physical AI & Humanoid Robotics Interactive Textbook codebase on 2026-06-26. This intelligence object is designed to be reusable across projects implementing RAG chat systems, LLM-powered APIs, and persistent chat UIs.*
