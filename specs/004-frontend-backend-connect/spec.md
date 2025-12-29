# Feature Specification: Connect Frontend ↔ Backend with ChatKit

**Feature Branch**: `main`
**Created**: 2025-12-25
**Status**: Draft
**Input**: User description: "Create a new folder inside `specs` for Spec-3: Connect (Frontend ↔ Backend).

Context:
- Frontend: Docusaurus (localhost:3000)
- Chat UI: Official ChatKit SDK https://platform.openai.com/docs/guides/chatkit (UI only, no backend usage)
- Backend: FastAPI (localhost:8000)
- Agent: OpenAI Agents SDK v0.6 (Gemini adapter)
- Vector DB: Qdrant (already populated)
- Book content ingested and retrievable

Objective:
Connect the existing backend RAG agent with the book UI using ChatKit as a frontend-only interface.

Requirements:
- Expose a single POST `/chat` endpoint
- Request payload:
  - message: string
  - selected_text: string | null
- If `selected_text` exists:
  - Ignore vector retrieval
  - Answer strictly from selected_text
- Else:
  - Retrieve top-K (default 5) relevant chunks from Qdrant
- If no relevant chunks found in Qdrant, return appropriate response indicating no relevant content found
- Implement timeout (30s default with configurable value) and retry mechanisms (3 attempts with exponential backoff) for agent and Qdrant calls
- Provide resolved context to the Agent
- Generate grounded, book-only answers with proper source attribution
- Politely refuse off-topic queries (no HTTP errors)
- Handle empty/malformed messages gracefully with appropriate responses
- Preserve and return metadata (module, chapter, section, version) for retrieved content
- Ensure response time under 5 seconds for 95% of requests under normal load conditions

Constraints:
- ChatKit is used strictly for UI and message transport
- ChatKit backend / hosted agents are not used
- Agent responses must maintain academic reliability and precise citations as per project constitution
- Error responses must maintain academic tone and precision
"

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.

  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - Chat with Book Content via UI (Priority: P1)

As a user reading the book content in the Docusaurus frontend, I want to ask questions about the book content through the ChatKit interface so that I can get contextually relevant answers based on the book material.

**Why this priority**: This is the core functionality that enables users to interact with the book content using AI assistance, providing the primary value proposition of the feature.

**Independent Test**: Can be fully tested by sending a message through the ChatKit UI to the backend and receiving a response that is grounded in the book content, delivering contextual assistance to the user.

**Acceptance Scenarios**:

1. **Given** user is viewing book content and has access to ChatKit interface, **When** user submits a question about the book, **Then** user receives an answer grounded in the book content
2. **Given** user has selected text in the book content, **When** user asks a question about the selected text, **Then** the response is generated strictly from the selected text without vector retrieval

---

### User Story 2 - Context-Aware Responses (Priority: P2)

As a user, I want the chat system to understand when I'm referring to specific selected text so that I can get more precise answers about that particular content.

**Why this priority**: This provides enhanced user experience by allowing users to ask specific questions about selected portions of text, making the interaction more precise and useful.

**Independent Test**: Can be tested by providing selected text in the request and verifying that the response is based solely on that text rather than general book content.

**Acceptance Scenarios**:

1. **Given** user has selected text and asks a question, **When** request includes the selected_text field, **Then** response is generated only from the provided selected text

---

### User Story 3 - Polite Query Handling (Priority: P3)

As a user, I want the system to politely decline off-topic questions so that I understand the scope of the AI assistant without encountering errors.

**Why this priority**: This provides a good user experience by managing expectations about the AI's capabilities while maintaining a professional interaction.

**Independent Test**: Can be tested by submitting off-topic queries and verifying that the system responds with a polite refusal rather than an error.

**Acceptance Scenarios**:

1. **Given** user submits an off-topic query, **When** the query is processed by the RAG agent, **Then** the system responds politely explaining its scope limitations

---

## Requirements *(mandatory)*

<!--
  ACTION REQUIRED: The content in this section represents placeholders.
  Fill them out with the right functional requirements.
-->

### Functional Requirements

- **FR-001**: System MUST expose a single POST `/chat` endpoint for communication between frontend and backend
- **FR-002**: System MUST accept request payload with `message` (string) and `selected_text` (string | null) fields
- **FR-003**: When `selected_text` exists in request, system MUST ignore vector retrieval and answer strictly from the provided selected_text
- **FR-004**: When `selected_text` is null, system MUST retrieve top-K (default K=5) relevant chunks from Qdrant vector database
- **FR-005**: When Qdrant returns zero results, system MUST return appropriate response indicating no relevant content found in the book
- **FR-006**: System MUST implement timeout (30s default with configurable value) and retry mechanisms (3 attempts with exponential backoff) for agent and Qdrant calls
- **FR-007**: System MUST provide resolved context to the Agent (OpenAI Agents SDK with Gemini adapter)
- **FR-008**: System MUST generate grounded, book-only answers with proper source attribution to maintain academic reliability
- **FR-009**: System MUST politely refuse off-topic queries without returning HTTP errors
- **FR-010**: System MUST handle empty/malformed messages gracefully with appropriate responses
- **FR-011**: System MUST preserve and return metadata (module, chapter, section, version) for retrieved content
- **FR-012**: System MUST use ChatKit strictly for UI and message transport, not for backend processing
- **FR-013**: System MUST integrate with existing FastAPI backend running on localhost:8000
- **FR-014**: System MUST work with existing Docusaurus frontend running on localhost:3000
- **FR-015**: System MUST implement rate limiting to prevent abuse (max 100 requests per minute per IP address)

### Key Entities *(include if feature involves data)*

- **ChatRequest**: Represents a user's chat input with message and optional selected_text for context
- **ChatResponse**: Represents the AI-generated response to the user's query, including sources with metadata (module, chapter, section, version)
- **Context**: The resolved context provided to the Agent, either from selected_text or retrieved from Qdrant with metadata
- **ChunkMetadata**: Contains book content metadata including module, chapter, section, and version information for proper attribution

## Edge Cases

- What happens when user sends empty message? - System responds with appropriate guidance message within 2 seconds
- What happens when Qdrant returns zero relevant chunks? - System responds with "No relevant content found in the book" message within 3 seconds
- How does system handle malformed JSON requests? - System returns appropriate error response without crashing
- What happens during network timeouts to Qdrant or Agent services? - System implements retry mechanism (3 attempts) then returns graceful error
- How does system handle extremely long messages? - System validates and truncates if necessary, with appropriate response
- What happens when selected_text and Qdrant both provide content? - Selected_text always takes precedence, Qdrant ignored
- How does system handle potential security threats? - System validates and sanitizes all user inputs, implements rate limiting, and prevents injection attacks
- What happens when system faces abuse attempts? - System implements rate limiting (max 100 requests per minute per IP) and blocks suspicious activity

## Success Criteria *(mandatory)*

<!--
  ACTION REQUIRED: Define measurable success criteria.
  These must be technology-agnostic and measurable.
-->

### Measurable Outcomes

- **SC-001**: 95% of API requests return responses within 5 seconds under normal load conditions (1-10 concurrent users, with 95th percentile response time measured during 8-hour production-like testing period)
- **SC-002**: 95% of on-topic queries receive relevant, book-grounded responses that satisfy the user's information needs
- **SC-003**: 100% of off-topic queries receive polite refusal responses without system errors
- **SC-004**: When selected_text is provided, 100% of responses are generated strictly from that text without vector retrieval
- **SC-005**: System maintains 99% uptime during normal usage periods
- **SC-006**: 100% of responses include proper source attribution with metadata (module, chapter, section) when content is retrieved from Qdrant
- **SC-007**: 100% of requests with malformed/empty messages receive appropriate error responses within 2 seconds
- **SC-008**: When Qdrant returns zero relevant results, system responds with appropriate "no relevant content found" message within 3 seconds
- **SC-009**: System enforces rate limiting (max 100 requests per minute per IP address) to prevent abuse while maintaining legitimate user access