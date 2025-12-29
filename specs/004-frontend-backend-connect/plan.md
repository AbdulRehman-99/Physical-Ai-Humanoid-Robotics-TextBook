# Implementation Plan: Connect Frontend ↔ Backend with ChatKit

**Feature**: Connect Frontend ↔ Backend with ChatKit
**Created**: 2025-12-25
**Status**: Draft
**Author**: Claude
**Branch**: main

## Technical Context

This feature implements a connection between the Docusaurus frontend and FastAPI backend using ChatKit as a frontend-only layer. The architecture follows this flow: Docusaurus → ChatKit → FastAPI `/chat` → Agent → (selected_text or Qdrant) → Gemini → ChatKit.

### Architecture Overview

- **Frontend**: Docusaurus (localhost:3000) with ChatKit integration
- **Transport**: ChatKit handles UI and message transport only
- **Backend**: FastAPI (localhost:8000) with RAG agent
- **Agent**: OpenAI Agents SDK v0.6 (Gemini adapter)
- **Vector DB**: Qdrant (already populated with book content)
- **Core Logic**: Context switching between selected_text and Qdrant retrieval

### Key Technical Decisions

1. **selected_text override**: When provided, always bypasses vector retrieval
2. **ChatKit role**: UI and transport only; no backend processing
3. **Single endpoint**: POST `/chat` handles all chat interactions
4. **Guardrails**: Enforced at the Agent level for off-topic queries
5. **Response format**: Compatible with ChatKit's expected format
6. **Zero-results fallback**: When Qdrant returns no relevant chunks, return appropriate "no content found" response
7. **Timeout/retry mechanisms**: Implement 30s timeout (configurable) with 3 retry attempts and exponential backoff for agent and Qdrant calls
8. **Metadata preservation**: Preserve and return book content metadata (module, chapter, section, version) with responses
9. **Academic reliability**: Ensure agent responses maintain academic reliability and precise citations per constitution

### Dependencies

- FastAPI (backend framework)
- OpenAI Agents SDK v0.6 with Gemini adapter
- Qdrant client for vector database access
- ChatKit SDK for frontend integration
- Existing RAG infrastructure
- Rate limiting library (for preventing abuse)

### Unknowns

- None - all clarifications resolved in research.md

## Constitution Check

### Alignment with Project Principles

- ✅ **Module Alignment**: Supports the Physical AI & Humanoid Robotics course by providing interactive access to book content
- ✅ **Academic Reliability**: RAG system ensures responses are grounded in verified book content
- ✅ **Terminology Consistency**: Will maintain consistent terminology through the RAG system
- ✅ **Precise Citations**: Agent responses will be based on properly indexed book content
- ✅ **Clarity and Reproducibility**: Clear API contracts and documented integration points
- ✅ **Approved Toolchain**: Uses Docusaurus and FastAPI as approved technologies
- ✅ **Accessibility and Structure**: Chat interface will be accessible and well-structured

### Potential Violations

- **Academic Reliability**: Using OpenAI Agent SDK with Gemini adapter may risk compromising academic reliability if the agent doesn't properly maintain source attribution and precise citations. Mitigation: Implement strict validation of agent responses to ensure they reference only book content with proper citations.
- **Precise Citations**: Need to ensure agent responses include proper citations to book content. Mitigation: Include metadata in responses with module, chapter, section, and version information.

## Phase 0: Research

### Research Tasks

1. **Qdrant Integration Research**
   - Determine collection schema for book content
   - Understand query parameters for retrieval
   - Identify top-K configuration (default K=5) for relevant chunks
   - Research zero-results handling and fallback strategies
   - Understand metadata schema for book content (module, chapter, section, version)

2. **ChatKit Integration Research**
   - Understand request/response format expectations
   - Determine how to integrate with existing Docusaurus setup
   - Research error handling patterns
   - Validate payload validation requirements

3. **OpenAI Agent Configuration Research**
   - Identify proper configuration for Gemini adapter
   - Understand context window limitations
   - Research guardrail implementation patterns
   - Investigate academic reliability and citation requirements
   - Determine timeout and retry configuration options

4. **Error Handling & Performance Research**
   - Research timeout and retry mechanisms for API calls
   - Investigate graceful error handling strategies
   - Understand performance optimization techniques
   - Research rate limiting strategies and implementation options

5. **Security Research**
   - Investigate input validation and sanitization techniques
   - Research security best practices for user inputs
   - Understand potential injection attack prevention methods

## Phase 1: Design

### Backend Architecture

#### Connection Module Structure
```
Backend/
├── Connection/
│   ├── __init__.py
│   ├── api.py                 # FastAPI endpoints
│   ├── context_switcher.py    # Logic for selected_text vs Qdrant
│   ├── agent_wrapper.py       # Agent invocation and configuration
│   ├── response_formatter.py  # ChatKit-compatible response formatting
│   └── models.py              # Data models for requests/responses
```

#### Data Models

**ChatRequest**
- message: string (user's question)
- selected_text: string | null (optional context from UI)

**ChatResponse**
- response: string (AI-generated answer)
- sources: array (references to book content with metadata, if applicable)

**ChunkMetadata**
- module: string (book module identifier)
- chapter: string (chapter identifier)
- section: string (section identifier)
- version: string (content version)

#### API Contract

**POST /chat**
- Request: ChatRequest model
- Response: ChatResponse model
- Error handling: Polite refusal for off-topic queries

### Integration Points

1. **Frontend Integration**: ChatKit will send requests to the `/chat` endpoint
2. **RAG Integration**: Existing Qdrant retrieval system will be reused
3. **Agent Integration**: OpenAI Agent SDK with Gemini adapter will process queries
4. **Guardrail Integration**: Agent-level off-topic detection and response

## Phase 2: Implementation Approach

### Module 1: Connection API Layer
- Create FastAPI router for `/chat` endpoint
- Implement request validation and parsing
- Set up error handling middleware
- Add timeout and retry configuration (30s timeout, 3 attempts with exponential backoff)
- Implement validation for empty/malformed messages
- Add rate limiting middleware (max 100 requests per minute per IP)
- Implement input sanitization for security

### Module 2: Context Switcher
- Implement logic to choose between selected_text and Qdrant retrieval
- Ensure selected_text always overrides vector retrieval when present
- Validate context quality before passing to agent
- Handle zero-results case from Qdrant with appropriate fallback

### Module 3: Agent Wrapper
- Configure OpenAI Agent SDK with Gemini adapter
- Implement proper context passing to agent
- Handle agent response processing
- Add timeout and retry mechanisms for agent calls
- Ensure academic reliability and proper citation in responses

### Module 4: Response Formatter
- Format agent responses for ChatKit compatibility
- Ensure proper error response formatting
- Add source attribution with metadata (module, chapter, section, version)
- Handle special cases like zero-results and timeout scenarios

## Phase 3: Testing Strategy

### Unit Tests
- Context switcher logic validation
- API request/response validation
- Error handling scenarios
- Timeout and retry mechanisms
- Empty/malformed message handling
- Zero-results fallback validation
- Metadata preservation testing
- Rate limiting functionality validation
- Input sanitization and security validation

### Integration Tests
- Full request flow from API to agent and back
- selected_text override behavior
- Qdrant retrieval fallback
- ChatKit compatibility validation
- Zero-results handling end-to-end
- Timeout and retry behavior
- Academic reliability validation
- Rate limiting enforcement
- Security vulnerability testing

### End-to-End Tests
- Complete UI to backend flow
- Off-topic query handling
- Performance under load
- Empty/malformed message scenarios
- Zero-results fallback scenarios
- Metadata preservation validation
- Rate limiting effectiveness
- Security penetration testing

## Risk Assessment

### High-Risk Areas
- Agent response quality and consistency (academic reliability)
- Qdrant retrieval performance and zero-results handling
- ChatKit integration compatibility
- Timeout and retry mechanism failures
- Metadata preservation accuracy
- Academic reliability and citation accuracy
- Security vulnerabilities from user inputs
- Rate limiting effectiveness and abuse prevention

### Mitigation Strategies
- Implement comprehensive error handling
- Add response validation and quality checks
- Include fallback mechanisms for different failure modes
- Implement proper timeout and retry mechanisms
- Add metadata validation and attribution checks
- Include academic reliability validation in responses
- Implement input sanitization and security validation
- Add rate limiting and abuse detection mechanisms
- Conduct security testing and vulnerability assessments

## Success Criteria

### Technical Metrics
- API response time under 5 seconds for 95% of requests under normal load (1-10 concurrent users, with 95th percentile response time measured during 8-hour production-like testing period)
- 99% uptime during normal operation
- Proper handling of 100% of off-topic queries
- Timeout handling with 30s default (configurable) and 3 retry attempts with exponential backoff for all external calls
- Empty/malformed message handling with response within 2 seconds
- Zero-results from Qdrant handled with appropriate response within 3 seconds
- Rate limiting enforced at max 100 requests per minute per IP address to prevent abuse

### Functional Metrics
- 100% of selected_text requests bypass Qdrant retrieval
- 95% of on-topic queries receive relevant, book-grounded responses
- Zero HTTP errors for any user request
- 100% of responses include proper source attribution with metadata when content is retrieved
- 100% of agent responses maintain academic reliability and proper citations
- 100% of user inputs properly sanitized to prevent security vulnerabilities

## Implementation Timeline

### Week 1: Core Infrastructure
- Set up Connection module structure
- Implement basic API endpoint
- Integrate with existing RAG system

### Week 2: Context Logic & Agent Integration
- Implement context switching logic
- Integrate with OpenAI Agent SDK
- Add response formatting

### Week 3: Testing & Validation
- Comprehensive testing of all scenarios
- Performance optimization
- Error handling refinement

### Week 4: Integration & Deployment
- End-to-end testing with frontend
- Deployment preparation
- Documentation completion