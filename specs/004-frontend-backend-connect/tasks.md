---
description: "Task list for frontend-backend connection using ChatKit"
---

# Tasks: Connect Frontend ↔ Backend with ChatKit

**Input**: Design documents from `/specs/004-frontend-backend-connect/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `Backend/` with `Connection/` module
- **API**: `Backend/Connection/api.py`
- **Models**: `Backend/Connection/models.py`
- **Services**: `Backend/Connection/` for context_switcher, agent_wrapper, response_formatter
- **Dependencies**: `Backend/requirements.txt`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create Connection module structure in Backend/Connection/
- [x] T002 [P] Initialize Python project with FastAPI dependencies in Backend/Connection/requirements.txt
- [x] T003 [P] Create __init__.py files for Connection module
- [x] T004 Set up environment configuration for Qdrant and OpenAI API in Backend/.env

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T005 Create data models for ChatRequest, ChatResponse, ChatContext, and ChunkMetadata in Backend/Connection/models.py
- [x] T006 [P] Set up FastAPI application structure in Backend/Connection/api.py
- [x] T007 [P] Configure Qdrant client for vector database access in Backend/Connection/
- [x] T008 Configure OpenAI Agent SDK with timeout and retry settings in Backend/Connection/
- [x] T009 Set up error handling and logging infrastructure in Backend/Connection/
- [x] T010 Create API endpoint contract for POST /chat in Backend/Connection/api.py
- [x] T011 [P] Implement timeout and retry configuration for Qdrant and Agent calls in Backend/Connection/config.py
- [x] T012 [P] Implement rate limiting configuration (max 100 requests per minute per IP) in Backend/Connection/config.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Chat with Book Content via UI (Priority: P1) 🎯 MVP

**Goal**: Enable users to ask questions about book content through ChatKit interface and receive responses grounded in book content

**Independent Test**: Can be fully tested by sending a message through the ChatKit UI to the backend and receiving a response that is grounded in the book content, delivering contextual assistance to the user.

### Tests for User Story 1 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**


### Implementation for User Story 1

- [x] T012 [P] [US1] Implement basic POST /chat endpoint with timeout/retry middleware in Backend/Connection/api.py
- [x] T013 [US1] Implement Qdrant retrieval logic with zero-results handling in Backend/Connection/retrieval.py
- [x] T014 [US1] Implement basic agent invocation with timeout/retry in Backend/Connection/agent_wrapper.py
- [x] T015 [US1] Create response formatter with metadata support in Backend/Connection/response_formatter.py
- [x] T016 [US1] Add validation for ChatRequest model including malformed/empty messages in Backend/Connection/models.py
- [x] T017 [US1] Add basic error handling for off-topic queries with academic reliability in Backend/Connection/api.py
- [x] T018 [US1] Implement zero-results fallback response in Backend/Connection/retrieval.py
- [x] T019 [US1] Add rate limiting middleware (max 100 requests per minute per IP) in Backend/Connection/api.py
- [x] T020 [US1] Implement input sanitization for security in Backend/Connection/api.py

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Context-Aware Responses (Priority: P2)

**Goal**: Enable the chat system to understand when users are referring to specific selected text and provide more precise answers about that particular content

**Independent Test**: Can be tested by providing selected text in the request and verifying that the response is based solely on that text rather than general book content.


### Implementation for User Story 2

- [x] T021 [P] [US2] Implement context switcher logic with metadata handling in Backend/Connection/context_switcher.py
- [x] T022 [US2] Update POST /chat endpoint to use context switcher in Backend/Connection/api.py
- [x] T023 [US2] Implement selected_text validation with academic reliability in Backend/Connection/models.py
- [x] T024 [US2] Add logic to bypass Qdrant retrieval when selected_text exists in Backend/Connection/context_switcher.py
- [x] T025 [US2] Update response formatter to handle selected_text context with metadata in Backend/Connection/response_formatter.py

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Polite Query Handling (Priority: P3)

**Goal**: Ensure the system politely declines off-topic questions so users understand the scope of the AI assistant without encountering errors

**Independent Test**: Can be tested by submitting off-topic queries and verifying that the system responds with a polite refusal rather than an error.


### Implementation for User Story 3

- [x] T026 [P] [US3] Implement guardrail logic with academic reliability in Backend/Connection/agent_wrapper.py
- [x] T027 [US3] Update agent configuration to ensure proper citations in Backend/Connection/agent_wrapper.py
- [x] T028 [US3] Create polite refusal message templates with academic tone in Backend/Connection/
- [x] T029 [US3] Update response formatter to ensure academic reliability in off-topic responses in Backend/Connection/response_formatter.py
- [x] T030 [US3] Add comprehensive error handling with proper academic tone in Backend/Connection/api.py

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T031 [P] Add comprehensive logging across all modules in Backend/Connection/
- [x] T032 [P] Performance optimization for Qdrant retrieval and agent calls with timeout handling
- [x] T033 [P] Add request/response validation middleware in Backend/Connection/api.py
- [x] T034 [P] Add rate limiting to prevent abuse in Backend/Connection/api.py
- [x] T035 [P] Add comprehensive documentation for metadata handling in Backend/Connection/README.md
- [x] T036 [P] Add environment configuration validation for timeout/retry settings in Backend/Connection/
- [x] T037 [P] Add health check endpoint with timeout/retry validation in Backend/Connection/api.py
- [x] T038 Run end-to-end validation with frontend integration including zero-results scenarios
- [x] T039 Add academic reliability validation in system-wide response processing
- [x] T040 Add metadata preservation validation across all response flows
- [x] T041 Add security validation and input sanitization checks across all modules

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together (if tests requested):
Task: "Contract test for POST /chat endpoint in Backend/Connection/tests/test_contract.py"
Task: "Integration test for basic chat flow in Backend/Connection/tests/test_integration.py"

# Launch all models for User Story 1 together:
Task: "Implement basic POST /chat endpoint in Backend/Connection/api.py"
Task: "Implement Qdrant retrieval logic in Backend/Connection/retrieval.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence