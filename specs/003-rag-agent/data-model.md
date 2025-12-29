# Data Model: RAG Agent for Book Content

## Entities

### ChatMessage
**Description**: Represents a user message with optional selected text, containing the query and context information

**Fields**:
- `message: str` - The user's query message
- `selected_text: Optional[str]` - Optional selected text from the UI (nullable)
- `timestamp: datetime` - When the message was created
- `user_id: Optional[str]` - Identifier for the user (if available)

**Validation Rules**:
- `message` must not be empty
- `selected_text` must not exceed 10,000 characters if provided
- `message` should be less than 10,000 characters

### RAGResponse
**Description**: Represents the agent's response, grounded in book content or selected text

**Fields**:
- `response: str` - The agent's response to the user's query
- `source_chunks: List[str]` - The source chunks used to generate the response (when applicable)
- `confidence: float` - Confidence score of the response (0.0 to 1.0)
- `timestamp: datetime` - When the response was generated

**Validation Rules**:
- `response` must not be empty
- `source_chunks` must be empty when using selected_text context
- `confidence` must be between 0.0 and 1.0

### ContextSource
**Description**: Represents the source of information for the agent (vector retrieval or user-selected text)

**Fields**:
- `type: Literal["selected_text", "vector_retrieval", "llm_only"]` - The type of context source used
- `content: str` - The actual context content used
- `retrieved_chunks_count: int` - Number of chunks retrieved (for vector retrieval)
- `metadata: Dict[str, Any]` - Additional metadata about the context source

**Validation Rules**:
- `type` must be one of the allowed literal values
- `content` must not be empty
- `retrieved_chunks_count` must be non-negative

### AgentSession
**Description**: Represents a session with the RAG agent

**Fields**:
- `session_id: str` - Unique identifier for the session
- `created_at: datetime` - When the session was created
- `last_interaction: datetime` - When the last interaction occurred
- `context_history: List[Dict]` - History of context used in the session

**Validation Rules**:
- `session_id` must be unique
- `created_at` must be before `last_interaction`

## Relationships

- One `AgentSession` can contain multiple `ChatMessage` instances
- One `ChatMessage` generates one `RAGResponse`
- One `RAGResponse` is associated with one `ContextSource`

## State Transitions

### ChatMessage
- `pending` → `processing` → `completed` (or `failed`)

### AgentSession
- `active` → `inactive` (after timeout period)