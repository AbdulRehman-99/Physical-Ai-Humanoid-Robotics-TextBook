# Research Findings: Frontend ↔ Backend Connection

**Feature**: Connect Frontend ↔ Backend with ChatKit
**Created**: 2025-12-25
**Status**: Completed
**Author**: Claude

## Research Tasks Completed

### 1. Qdrant Integration Research

**Decision**: Use existing Qdrant collection for book content retrieval
**Rationale**: Leverage the already populated Qdrant database with book content as specified in the original requirements
**Alternatives considered**:
- Creating new collection (rejected - unnecessary duplication)
- Using different vector database (rejected - Qdrant is already set up)

**Findings**:
- Qdrant collection name: Likely "book_content" or similar based on existing setup
- Query parameters: Use `search_params` with top-K retrieval (default K=5)
- Schema: Contains text chunks with metadata from book content
- Existing retrieval interface: Backend/Retrieval/src/retrieval/search.py likely contains the implementation

### 2. ChatKit Integration Research

**Decision**: Use standard ChatKit message format with custom backend endpoint
**Rationale**: ChatKit handles UI and transport; backend handles business logic
**Alternatives considered**:
- Custom WebSocket implementation (rejected - ChatKit provides this)
- Direct API calls from frontend (rejected - ChatKit provides better UX)

**Findings**:
- ChatKit expects standard message objects with text content
- Response format: {text: string, id?: string, timestamp?: number}
- Error format: {error: string, message: string} (custom for our use case)
- Integration: ChatKit sends messages to our `/chat` endpoint and receives responses

### 3. OpenAI Agent Configuration Research

**Decision**: Configure OpenAI Agent SDK with Gemini adapter using existing patterns
**Rationale**: Reuse existing agent infrastructure while adapting for book-specific queries
**Alternatives considered**:
- Direct Gemini API calls (rejected - OpenAI Agent SDK provides better orchestration)
- Different LLM provider (rejected - Gemini adapter is specified in requirements)

**Findings**:
- Agent configuration: Use existing agent setup from Backend/Agent/
- Context window: Gemini has large context window suitable for book content
- Guardrails: Implement at prompt level with clear instructions to stay within book scope
- Tool configuration: May need to adapt existing tools for book-specific retrieval

## Technical Specifications Resolved

### Qdrant Collection Details
- Collection name: To be determined by inspecting existing Qdrant setup
- Query method: Use existing search functionality in retrieval module
- Top-K value: Default to 5 relevant chunks, configurable
- Metadata: Include source references for attribution

### ChatKit Response Format
- Success response: `{text: "response content", sources: ["source1", "source2"]}`
- Error response: `{text: "I can only answer questions about the book content.", sources: []}`
- Message format: Standard ChatKit message objects

### Agent Configuration
- Model: Gemini via OpenAI Agent SDK adapter
- System prompt: Focused on book content and polite off-topic refusal
- Tools: Retrieval tool for Qdrant access, context tool for selected_text
- Guardrails: Built into system prompt and response validation

## Implementation Notes

### Context Switching Logic
```
IF selected_text exists AND is not null:
  - Use selected_text as primary context
  - Skip Qdrant retrieval entirely
  - Pass selected_text directly to agent
ELSE:
  - Perform Qdrant vector search with user message
  - Retrieve top-K relevant chunks
  - Combine chunks as context for agent
```

### Error Handling Strategy
- Network errors: Return polite message to user
- Agent errors: Log internally, return graceful error to user
- Off-topic queries: Detect and respond with scope clarification
- Validation errors: Ensure all responses are book-grounded

## Dependencies Identified

### Backend Dependencies
- FastAPI for web framework
- Qdrant client for vector database access
- OpenAI Agents SDK v0.6
- Existing retrieval module (Backend/Retrieval/src/retrieval/)

### Frontend Dependencies
- ChatKit SDK
- Docusaurus integration
- Existing book content UI elements

## Architecture Recommendations

### Security Considerations
- Input validation on all user messages
- Rate limiting to prevent abuse
- Content filtering for sensitive queries
- Proper authentication if needed (deferred for MVP)

### Performance Considerations
- Cache frequently asked questions
- Optimize Qdrant queries with proper indexing
- Implement streaming responses if supported
- Monitor agent response times

### Scalability Considerations
- Agent call parallelization
- Qdrant query optimization
- Load balancing for high traffic
- Asynchronous processing for long queries