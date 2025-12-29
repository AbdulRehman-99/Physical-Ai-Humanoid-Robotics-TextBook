# Feature Specification: RAG Agent for Book Content

**Feature Branch**: `003-rag-agent`
**Created**: 2025-12-23
**Status**: Draft
**Input**: User description: "Build a backend RAG agent that answers user questions strictly from book content or user-selected text, with no external knowledge."

## Clarifications

### Session 2025-12-23

- Q: What is the responsibility separation between ChatKit and backend regarding selected text processing? → A: ChatKit only handles UI rendering and sends raw selected text to backend for processing
- Q: What is the maximum size limit for selected_text and how should the system behave when exceeded? → A: Backend enforces a 10,000 character limit on selected_text with clear error response
- Q: How should the system respond when the vector database is temporarily unavailable? → A: System attempts to answer from LLM without vector context when database is unavailable
- Q: What is the expected concurrent user capacity the system should support? → A: System should support up to 100 concurrent users with acceptable response times
- Q: Are there specific security or privacy requirements for handling user data or book content? → A: System must not store user questions or selected text persistently, only in-memory processing

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask Book-Related Questions (Priority: P1)

A student wants to ask questions about book content through a chat interface. The user types a question in the chat, optionally selects specific text on the page, and expects to receive an accurate answer based only on the book content or selected text.

**Why this priority**: This is the core functionality that delivers the main value of the RAG system - helping users understand book content through AI-powered Q&A.

**Independent Test**: Can be fully tested by sending questions to the chat endpoint and verifying that responses are grounded in book content without external knowledge.

**Acceptance Scenarios**:

1. **Given** book content is available in the vector database, **When** user asks a question about the book content, **Then** agent responds with accurate information based only on the book content
2. **Given** user has selected specific text in the UI, **When** user asks a question with selected_text parameter, **Then** agent responds based only on the selected text, ignoring vector retrieval

---

### User Story 2 - Handle Off-Topic Queries (Priority: P2)

A user asks questions that are not related to the book content. The system should gracefully decline to answer and guide the user back to book-related topics.

**Why this priority**: Important for maintaining the integrity of the RAG system and preventing hallucinations or external knowledge usage.

**Independent Test**: Can be tested by sending off-topic queries and verifying that the system declines appropriately without errors.

**Acceptance Scenarios**:

1. **Given** user asks an off-topic question, **When** agent receives the query, **Then** agent responds with a polite refusal to answer external knowledge questions

---

### User Story 3 - Integrate with ChatKit UI (Priority: P3)

The chat interface uses the official ChatKit SDK to communicate with the backend. Messages flow seamlessly between the UI and the RAG agent backend.

**Why this priority**: Critical for the user experience and ensuring compatibility with the required technology stack.

**Independent Test**: Can be tested by connecting the ChatKit UI to the backend endpoint and verifying message flow.

**Acceptance Scenarios**:

1. **Given** ChatKit UI is connected to backend, **When** user sends a message, **Then** message is properly received by the backend endpoint

---

## Edge Cases

- What happens when no book content matches the query?
- How does the system handle malformed requests?
- How does the system handle very long selected text?
- What happens when the vector database is temporarily unavailable?
- How does the system handle concurrent users?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST expose a single POST /chat endpoint compatible with ChatKit
- **FR-002**: System MUST accept payload with structure { message: string, selected_text: string | null }
- **FR-003**: System MUST answer strictly from provided context (book content or selected text) when selected_text is provided (at least 90% of response content must be traceable to provided context)
- **FR-004**: System MUST embed query using Cohere when selected_text is null, retrieve top-K chunks from Qdrant, and use as context
- **FR-005**: System MUST safely refuse all off-topic/external knowledge queries without throwing HTTP errors
- **FR-006**: System MUST use OpenAI Agents SDK v0.6 for agent creation and execution
- **FR-007**: System MUST integrate with ChatKit SDK for UI rendering and message transport (ChatKit handles only UI rendering and sends raw selected text to backend)
- **FR-008**: System MUST use FastAPI as the backend framework
- **FR-009**: System MUST enforce a 10,000 character limit on selected_text parameter with appropriate error response when exceeded
- **FR-010**: System MUST attempt to answer from LLM without vector context when vector database is unavailable
- **FR-011**: System MUST not store user questions or selected text persistently, only in-memory processing
- **FR-012**: System MUST ensure academic reliability of responses by validating answers against verified book content sources

### Key Entities

- **ChatMessage**: Represents a user message with optional selected text, containing the query and context information
- **RAGResponse**: Represents the agent's response, grounded in book content or selected text
- **ContextSource**: Represents the source of information for the agent (vector retrieval or user-selected text)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Book-related questions return accurate, grounded answers with 90% accuracy based on manual review
- **SC-002**: Selected-text queries override vector retrieval 100% of the time in testing scenarios
- **SC-003**: Off-topic queries are consistently handled without HTTP errors in 100% of test cases
- **SC-004**: System responds to queries within 5 seconds for 95% of requests
- **SC-005**: Users rate the helpfulness of answers as 4+ stars out of 5 in satisfaction surveys
- **SC-006**: System supports up to 100 concurrent users with acceptable response times