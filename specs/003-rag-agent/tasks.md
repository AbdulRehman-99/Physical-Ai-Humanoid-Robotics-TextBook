# Testable Tasks: RAG Agent for Book Content

**Feature**: RAG Agent for Book Content
**Branch**: `003-rag-agent`
**Generated**: 2025-12-23
**Input**: specs/003-rag-agent/spec.md, specs/003-rag-agent/plan.md

## Implementation Strategy

MVP scope: Complete User Story 1 (core RAG functionality) with minimal viable endpoint that can handle both selected text and vector retrieval contexts. This provides the core value proposition of the system. Subsequent stories add guardrails and integration capabilities.

## Dependencies

- User Story 2 (off-topic handling) depends on User Story 1 (core functionality) being implemented first
- User Story 3 (ChatKit integration) can be implemented in parallel to User Stories 1 and 2 but requires the API endpoint to be stable

## Parallel Execution Examples

- Task T001-T004 (Setup phase) must complete before other phases
- Task T005-T010 (Foundational) can run in parallel with each other
- User Story 1 (T011-T020) can run in parallel with User Story 2 (T021-T025)
- User Story 3 (T026-T030) can run in parallel with other user stories after foundational tasks

---

## Phase 1: Setup

### Goal
Initialize project structure and dependencies following the technical plan.

### Independent Test Criteria
- Project directory structure exists as planned
- Dependencies can be installed successfully
- Basic environment is configured

### Tasks

- [X] T001 Create Backend/Agent directory structure
- [X] T002 Install uv package manager if not already installed
- [X] T003 Initialize Python 3.13 project with uv in Backend/Agent/
- [X] T004 Create pyproject.toml with dependencies: fastapi, uvicorn, openai-agents, litellm, cohere, qdrant-client, pydantic, python-dotenv

---

## Phase 2: Foundational

### Goal
Implement foundational components that all user stories depend on.

### Independent Test Criteria
- Settings module can load environment variables
- LLM adapter can connect to Gemini via LiteLLM
- Retrieval module can connect to Qdrant and Cohere
- Basic FastAPI app can start without errors

### Tasks

- [X] T005 [P] Create settings.py with configuration for API keys and service URLs
- [X] T006 [P] Create adapter.py with LiteLLM client factory for Gemini integration
- [X] T007 [P] Create retrieval.py with get_book_context function using Cohere embeddings and Qdrant
- [X] T008 [P] Create basic main.py with FastAPI app initialization
- [X] T009 [P] Create test_adapter.py to validate Agent SDK can handshake with Gemini
- [X] T010 [P] Create basic agent.py with skeleton Agent class
- [X] T011 [P] Add comprehensive error handling for Gemini API failures in adapter.py
- [X] T012 [P] Implement retry logic with exponential backoff for Gemini API calls
- [X] T013 [P] Add academic reliability validation function to verify response quality against book content

---

## Phase 3: [US1] Ask Book-Related Questions

### Goal
Implement core RAG functionality that can answer questions from book content or selected text.

### Independent Test Criteria
- Can send questions to the chat endpoint and receive responses grounded in book content
- When selected_text is provided, only that text is used for context
- When selected_text is null, vector retrieval is used for context
- Responses are generated without external knowledge

### Tasks

- [X] T021 [P] [US1] Implement ChatPayload Pydantic model in main.py with message and selected_text fields
- [X] T022 [P] [US1] Implement POST /chat endpoint in main.py with proper request/response handling
- [X] T023 [P] [US1] Create ContextSource class to handle context switching logic
- [X] T024 [P] [US1] Implement context switching: if selected_text exists use only that, else use vector retrieval
- [X] T025 [P] [US1] Integrate retrieval module to fetch book content when selected_text is null
- [X] T026 [P] [US1] Implement OpenAI Agent SDK v0.6 integration with context switching
- [X] T027 [P] [US1] Add 10,000 character limit validation for selected_text parameter
- [X] T028 [P] [US1] Implement response formatting compatible with ChatKit
- [X] T029 [P] [US1] Add error handling for character limit exceeded
- [X] T030 [P] [US1] Test core functionality with sample queries against book content

---

## Phase 4: [US2] Handle Off-Topic Queries

### Goal
Implement guardrails to safely refuse off-topic/external knowledge queries.

### Independent Test Criteria
- Off-topic questions receive polite refusal responses without HTTP errors
- System maintains integrity by not generating external knowledge
- Responses follow the same format as regular responses

### Tasks

- [X] T031 [P] [US2] Define system prompt instructions to refuse off-topic queries
- [X] T032 [P] [US2] Implement off-topic query detection in the agent
- [X] T033 [P] [US2] Create polite refusal response templates
- [X] T034 [P] [US2] Add off-topic handling to POST /chat endpoint
- [X] T035 [P] [US2] Test off-topic query handling with various external knowledge questions

---

## Phase 5: [US3] Integrate with ChatKit UI

### Goal
Ensure seamless integration with ChatKit SDK for UI rendering and message transport.

### Independent Test Criteria
- ChatKit UI can connect to backend endpoint
- Messages flow seamlessly between UI and backend
- Response format is compatible with ChatKit expectations

### Tasks

- [X] T036 [P] [US3] Verify response format compatibility with ChatKit requirements
- [X] T041 [P] [US3] Add CORS middleware to FastAPI app for browser-based ChatKit
- [X] T038 [P] [US3] Test backend endpoint with sample ChatKit UI integration
- [X] T039 [P] [US3] Implement any additional API fields required by ChatKit
- [X] T040 [P] [US3] Complete integration testing with ChatKit UI

---

## Phase 6: Polish & Cross-Cutting Concerns

### Goal
Add error handling, performance optimization, and security measures across the system.

### Independent Test Criteria
- System handles vector database unavailability gracefully
- No persistent storage of user queries (in-memory only)
- Performance meets 5-second response time requirement
- Error responses are user-friendly

### Tasks

- [X] T042 Implement fallback behavior when vector database is unavailable (use LLM without vector context)
- [X] T043 [P] Add comprehensive error handling for Qdrant vector database failures
- [X] T044 Ensure no persistent storage of user questions or selected text (in-memory only)
- [X] T045 Add comprehensive error handling and logging
- [X] T046 Implement rate limiting to support 100 concurrent users
- [X] T047 Add input validation for malformed requests
- [X] T048 Create health check endpoint
- [X] T049 Document API endpoints with OpenAPI/Swagger
- [X] T050 Add unit tests for critical components
- [X] T051 Deploy and test complete system integration
- [X] T052 [P] Add circuit breaker pattern for vector database calls to prevent cascade failures
- [X] T053 [P] Implement health monitoring for vector database connectivity
- [X] T054 [P] Add academic reliability validation to ensure response quality meets academic standards