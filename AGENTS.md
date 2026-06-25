# AGENTS.md

## Quick start

```powershell
# Frontend dev server (Docusaurus)
cd Frontend; npm start

# Backend orchestrator (starts Connection:8000 + Agent:8001 + Retrieval test)
cd Backend; uv run python main.py

# Hot-reload (Connection only)
$env:DEBUG="True"; uv run python app.py
# Or for all services via main.py: set DEBUG before running

# Single service
uv run python -m uvicorn Connection.api:app --port 8000

# Tests (Agent)
cd Backend/Agent; pytest

# Typecheck (Frontend)
cd Frontend; npm run typecheck
```

## Architecture

```
Frontend/  (Docusaurus 3.9.2, React 18, TS)
  ├── src/components/ChatKit/  — SSE streaming chat UI, localStorage multi-session, 48h auto-delete
  ├── static/img/              — 3 referenced + 12 unreferenced images
  └── assets/                  — 3 Gazebo/URDF sim files (unused in code)
Backend/   (Python 3.13, uv workspace)
  ├── Connection/              — Primary user-facing API (port 8000), consumed by ChatKit
  │   ├── api.py               — FastAPI app: POST /chat, POST /chat/stream, GET /health
  │   ├── agent_wrapper.py     — OpenRouter via OpenAIChatCompletionsModel, max_tokens=150, off-topic guardrail
  │   ├── guardrails.py        — LLM-based off-topic classification (@input_guardrail)
  │   ├── context_switcher.py  — uses sys.path hack to import Retrieval/src/
  │   └── config.py            — TOP_K=3, SESSION_MEMORY_TURNS=5, SESSION_TTL=1800s
  ├── Agent/                   — Standalone RAG Agent (port 8001), also sys.path-hacks Retrieval
  │   ├── main.py              — /chat + /chat/stream + /health endpoints
  │   ├── agent.py             — RAGAgent with process_message / process_message_streamed
  │   ├── adapter.py           — OpenRouterAdapter (shared client pattern)
  │   └── error_handling.py    — DEAD PRODUCTION CODE: only imported by test_core_functionality.py
  ├── Ingestion/               — Book → Qdrant pipeline (excluded from analysis)
  ├── Retrieval/               — Vector search service (excluded from analysis)
  │   └── src/retrieval/       — imported via sys.path hack, not as a pip package
  ├── main.py                  — Orchestrator: spawns 3 processes (retrieval test, Agent:8001, Connection:8000)
  └── app.py                   — HF Spaces entry point (port 7860, Connection only)
```

### Key design facts

- **Both services use OpenRouter** via `OpenAIChatCompletionsModel`, not the default Responses API (OpenRouter ignores `instructions` and `max_output_tokens` on `/v1/responses`).
- **max_tokens=150** everywhere — targets 80–100 word answers.
- **Off-topic guardrail at both layers**: Connection uses SDK `@input_guardrail` + manual check in streamed path; Agent uses its own `check_off_topic()` via ChatCompletions.
- **Session memory**: last 5 turns in-memory dict, 30-min idle TTL (Connection), UUID session_id.
- **ChatKit frontend**: SSE via `ReadableStream`, localStorage multi-session with 48h auto-delete, SSR-safe `try/catch` around all `localStorage` calls.
- **Docusaurus SSR constraint**: no `localStorage`/`window` in module scope — all browser-API calls need `try/catch`.
- **Conversation formatting**: plain text only (no markdown symbols), colon-label format for headers.

### Python import quirks

Both `Connection/context_switcher.py` and `Agent/agent.py` inject `Retrieval/src/` into `sys.path` before importing `from retrieval import ...`. This means:
- The Retrieval package is NOT importable by name unless the path hack runs first.
- Running `uvicorn Connection.api:app` from the `Backend/` root works because `Connection/__init__.py` triggers path manipulation via `context_switcher.py`.
- If a file is moved or a new service added, this sys.path hack must be replicated.

## Environment

| Variable | Where used | Required |
|---|---|---|
| `OPENROUTER_API_KEY` | Both services | Yes |
| `OPENROUTER_BASE_URL` | Both services | Yes (default: `https://openrouter.ai/api/v1`) |
| `OPENROUTER_MODEL` | Both services | Yes (default: `qwen-2.5-72b-instruct`) |
| `COHERE_API_KEY` | Ingestion/Agent | Yes |
| `QDRANT_API_KEY` | Ingestion/Retrieval | Yes |
| `QDRANT_URL` | Ingestion/Retrieval | Yes |
| `COLLECTION_NAME` | Ingestion/Retrieval | Yes (default: `book_embeddings`) |
| `TOP_K` | Both services | Default `3` |
| `DEBUG` | Any entry point | `True` enables hot-reload |

`.env` is loaded from the repo root. Both services search multiple parent directories.

## Deployment

- **Vercel** (frontend only): `vercel.json` runs `cd Frontend && npm install && npm run build`, output `Frontend/build`.
- **Hugging Face Spaces** (backend only): Dockerfile at `Backend/Dockerfile`, port 7860, entry `python app.py`.
- **Backend URL in frontend**: hardcoded in `ChatKit.tsx:14-18` (`BACKEND_URLS` array) and `docusaurus.config.ts:31` (`customFields.backendUrl`).

## Notable dead / unused code

- `Backend/Agent/error_handling.py` — CircuitBreaker + Qdrant error decorator, only imported by test file, not by any production code.
- `Frontend/static/img/` — 12 SVG/PNG files with zero references in source (including `favicon.ico`, `logo.svg`, 3 `undraw_docusaurus_*` defaults).
- `Frontend/assets/` — 3 Gazebo/URDF sim files (`default_world.sdf`, `humanoid_robot.urdf`, `vla_world.sdf`) not referenced in any code.

# opencode Rules — SDD Workflow

This section encodes the Spec-Driven Development (SDD) workflow: PHR creation, ADR suggestions, architectural planning, and execution contracts.

## Task context

**Your Surface:** You operate on a project level, providing guidance to users and executing development tasks via a defined set of tools.

**Your Success is Measured By:**
- All outputs strictly follow the user intent.
- Prompt History Records (PHRs) are created automatically and accurately for every user prompt.
- Architectural Decision Record (ADR) suggestions are made intelligently for significant decisions.
- All changes are small, testable, and reference code precisely.

## Core Guarantees (Product Promise)

- Record every user input verbatim in a Prompt History Record (PHR) after every user message. Do not truncate; preserve full multiline input.
- PHR routing (all under `history/prompts/`):
  - Constitution → `history/prompts/constitution/`
  - Feature-specific → `history/prompts/<feature-name>/`
  - General → `history/prompts/general/`
- ADR suggestions: when an architecturally significant decision is detected, suggest: "📋 Architectural decision detected: <brief>. Document? Run `/sp.adr <title>`." Never auto‑create ADRs; require user consent.

## Development Guidelines

### 1. Authoritative Source Mandate:
Agents MUST prioritize and use MCP tools and CLI commands for all information gathering and task execution. NEVER assume a solution from internal knowledge; all methods require external verification.

### 2. Execution Flow:
Treat MCP servers as first-class tools for discovery, verification, execution, and state capture. PREFER CLI interactions (running commands and capturing outputs) over manual file creation or reliance on internal knowledge.

### 3. Knowledge capture (PHR) for Every User Input.
After completing requests, you **MUST** create a PHR (Prompt History Record).

**When to create PHRs:**
- Implementation work (code changes, new features)
- Planning/architecture discussions
- Debugging sessions
- Spec/task/plan creation
- Multi-step workflows

**PHR Creation Process:**

1) Detect stage
   - One of: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate title
   - 3–7 words; create a slug for the filename.

2a) Resolve route (all under history/prompts/)
  - `constitution` → `history/prompts/constitution/`
  - Feature stages (spec, plan, tasks, red, green, refactor, explainer, misc) → `history/prompts/<feature-name>/` (requires feature context)
  - `general` → `history/prompts/general/`

3) Prefer agent‑native flow (no shell)
   - Read the PHR template from one of:
     - `.specify/templates/phr-template.prompt.md`
     - `templates/phr-template.prompt.md`
   - Allocate an ID (increment; on collision, increment again).
   - Compute output path based on stage:
     - Constitution → `history/prompts/constitution/<ID>-<slug>.constitution.prompt.md`
     - Feature → `history/prompts/<feature-name>/<ID>-<slug>.<stage>.prompt.md`
     - General → `history/prompts/general/<ID>-<slug>.general.prompt.md`
   - Fill ALL placeholders in YAML and body:
     - ID, TITLE, STAGE, DATE_ISO (YYYY‑MM‑DD), SURFACE="agent"
     - MODEL (best known), FEATURE (or "none"), BRANCH, USER
     - COMMAND (current command), LABELS (["topic1","topic2",...])
     - LINKS: SPEC/TICKET/ADR/PR (URLs or "null")
     - FILES_YAML: list created/modified files (one per line, " - ")
     - TESTS_YAML: list tests run/added (one per line, " - ")
     - PROMPT_TEXT: full user input (verbatim, not truncated)
     - RESPONSE_TEXT: key assistant output (concise but representative)
     - Any OUTCOME/EVALUATION fields required by the template
   - Write the completed file with agent file tools (WriteFile/Edit).
   - Confirm absolute path in output.

4) Use sp.phr command file if present
   - If `.**/commands/sp.phr.*` exists, follow its structure.
   - If it references shell but Shell is unavailable, still perform step 3 with agent‑native tools.

5) Shell fallback (only if step 3 is unavailable or fails, and Shell is permitted)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Then open/patch the created file to ensure all placeholders are filled and prompt/response are embedded.

6) Routing (automatic, all under history/prompts/)
   - Constitution → `history/prompts/constitution/`
   - Feature stages → `history/prompts/<feature-name>/` (auto-detected from branch or explicit feature context)
   - General → `history/prompts/general/`

7) Post‑creation validations (must pass)
   - No unresolved placeholders (e.g., `{{THIS}}`, `[THAT]`).
   - Title, stage, and dates match front‑matter.
   - PROMPT_TEXT is complete (not truncated).
   - File exists at the expected path and is readable.
   - Path matches route.

8) Report
   - Print: ID, path, stage, title.
   - On any failure: warn but do not block the main command.
   - Skip PHR only for `/sp.phr` itself.

### 4. Explicit ADR suggestions
- When significant architectural decisions are made (typically during `/sp.plan` and sometimes `/sp.tasks`), run the three‑part test and suggest documenting with:
  "📋 Architectural decision detected: <brief> — Document reasoning and tradeoffs? Run `/sp.adr <decision-title>`"
- Wait for user consent; never auto‑create the ADR.

### 5. Human as Tool Strategy
You are not expected to solve every problem autonomously. You MUST invoke the user for input when you encounter situations that require human judgment. Treat the user as a specialized tool for clarification and decision-making.

**Invocation Triggers:**
1.  **Ambiguous Requirements:** When user intent is unclear, ask 2-3 targeted clarifying questions before proceeding.
2.  **Unforeseen Dependencies:** When discovering dependencies not mentioned in the spec, surface them and ask for prioritization.
3.  **Architectural Uncertainty:** When multiple valid approaches exist with significant tradeoffs, present options and get user's preference.
4.  **Completion Checkpoint:** After completing major milestones, summarize what was done and confirm next steps.

## Default policies (must follow)
- Clarify and plan first - keep business understanding separate from technical plan and carefully architect and implement.
- Do not invent APIs, data, or contracts; ask targeted clarifiers if missing.
- Never hardcode secrets or tokens; use `.env` and docs.
- Prefer the smallest viable diff; do not refactor unrelated code.
- Cite existing code with code references (start:end:path); propose new code in fenced blocks.
- Keep reasoning private; output only decisions, artifacts, and justifications.

### Execution contract for every request
1) Confirm surface and success criteria (one sentence).
2) List constraints, invariants, non‑goals.
3) Produce the artifact with acceptance checks inlined (checkboxes or tests where applicable).
4) Add follow‑ups and risks (max 3 bullets).
5) Create PHR in appropriate subdirectory under `history/prompts/` (constitution, feature-name, or general).
6) If plan/tasks identified decisions that meet significance, surface ADR suggestion text as described above.

### Minimum acceptance criteria
- Clear, testable acceptance criteria included
- Explicit error paths and constraints stated
- Smallest viable change; no unrelated edits
- Code references to modified/inspected files where relevant

## Architect Guidelines (for planning)

Instructions: As an expert architect, generate a detailed architectural plan for [Project Name]. Address each of the following thoroughly.

1. Scope and Dependencies:
   - In Scope: boundaries and key features.
   - Out of Scope: explicitly excluded items.
   - External Dependencies: systems/services/teams and ownership.

2. Key Decisions and Rationale:
   - Options Considered, Trade-offs, Rationale.
   - Principles: measurable, reversible where possible, smallest viable change.

3. Interfaces and API Contracts:
   - Public APIs: Inputs, Outputs, Errors.
   - Versioning Strategy.
   - Idempotency, Timeouts, Retries.
   - Error Taxonomy with status codes.

4. Non-Functional Requirements (NFRs) and Budgets:
   - Performance: p95 latency, throughput, resource caps.
   - Reliability: SLOs, error budgets, degradation strategy.
   - Security: AuthN/AuthZ, data handling, secrets, auditing.
   - Cost: unit economics.

5. Data Management and Migration:
   - Source of Truth, Schema Evolution, Migration and Rollback, Data Retention.

6. Operational Readiness:
   - Observability: logs, metrics, traces.
   - Alerting: thresholds and on-call owners.
   - Runbooks for common tasks.
   - Deployment and Rollback strategies.
   - Feature Flags and compatibility.

7. Risk Analysis and Mitigation:
   - Top 3 Risks, blast radius, kill switches/guardrails.

8. Evaluation and Validation:
   - Definition of Done (tests, scans).
   - Output Validation for format/requirements/safety.

9. Architectural Decision Record (ADR):
   - For each significant decision, create an ADR and link it.

### Architecture Decision Records (ADR) - Intelligent Suggestion

After design/architecture work, test for ADR significance:

- Impact: long-term consequences? (e.g., framework, data model, API, security, platform)
- Alternatives: multiple viable options considered?
- Scope: cross‑cutting and influences system design?

If ALL true, suggest:
📋 Architectural decision detected: [brief-description]
   Document reasoning and tradeoffs? Run `/sp.adr [decision-title]`

Wait for consent; never auto-create ADRs. Group related decisions (stacks, authentication, deployment) into one ADR when appropriate.

## Basic Project Structure

- `.specify/memory/constitution.md` — Project principles
- `specs/<feature>/spec.md` — Feature requirements
- `specs/<feature>/plan.md` — Architecture decisions
- `specs/<feature>/tasks.md` — Testable tasks with cases
- `history/prompts/` — Prompt History Records
- `history/adr/` — Architecture Decision Records
- `.specify/` — SpecKit Plus templates and scripts

## Code Standards
See `.specify/memory/constitution.md` for code quality, testing, performance, security, and architecture principles.
