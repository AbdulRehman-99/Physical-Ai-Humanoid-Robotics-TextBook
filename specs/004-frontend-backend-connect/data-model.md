# Data Model: Frontend ↔ Backend Connection

**Feature**: Connect Frontend ↔ Backend with ChatKit
**Created**: 2025-12-25
**Status**: Draft
**Author**: Claude

## Entity Definitions

### ChatRequest
**Description**: Represents a user's chat input with message and optional selected text for context

**Fields**:
- `message` (string, required): The user's question or message to the AI
- `selected_text` (string | null, optional): Optional text selected by user in the book content for context

**Validation Rules**:
- `message` must not be empty or whitespace only
- `message` length should be between 1 and 2000 characters
- `selected_text` can be null or empty string (when no text is selected)

**Example**:
```json
{
  "message": "Explain the concept of inverse kinematics",
  "selected_text": "Inverse kinematics is the mathematical process of calculating the joint angles required to position the end effector of a robotic arm at a desired location."
}
```

### ChatResponse
**Description**: Represents the AI-generated response to the user's query

**Fields**:
- `response` (string, required): The AI-generated answer to the user's query
- `sources` (array of strings, optional): References to book content that informed the response
- `error` (string, optional): Error message if the request could not be processed

**Validation Rules**:
- `response` must not be empty when no error occurs
- `sources` should contain valid book content references when applicable
- `error` and `response` should not both be present

**Example**:
```json
{
  "response": "Inverse kinematics is the mathematical process of calculating the joint angles required to position the end effector of a robotic arm at a desired location. It's commonly used in robotics for motion planning.",
  "sources": ["Chapter 4: Robotic Motion Planning", "Section 4.2: Kinematics"]
}
```

### ChatContext
**Description**: Internal representation of the context provided to the AI agent

**Fields**:
- `user_message` (string, required): The original user message
- `retrieved_chunks` (array of strings, optional): Text chunks retrieved from Qdrant
- `selected_text` (string | null, optional): Text selected by user in the book content
- `effective_context` (string, required): The final context passed to the agent (either selected_text or retrieved chunks)

**Validation Rules**:
- `effective_context` must be populated based on priority: selected_text > retrieved_chunks
- If `selected_text` is present, `retrieved_chunks` should be ignored

**Example**:
```json
{
  "user_message": "Explain the concept of inverse kinematics",
  "retrieved_chunks": ["Inverse kinematics is the mathematical process...", "It's commonly used in robotics for motion planning..."],
  "selected_text": "Inverse kinematics is the mathematical process of calculating the joint angles required to position the end effector of a robotic arm at a desired location.",
  "effective_context": "Inverse kinematics is the mathematical process of calculating the joint angles required to position the end effector of a robotic arm at a desired location."
}
```

## API Contract

### POST /chat
**Description**: Endpoint to process user chat messages and return AI-generated responses

**Request**:
- Method: POST
- Path: /chat
- Content-Type: application/json
- Body: ChatRequest object

**Response**:
- Success: 200 OK with ChatResponse object
- Client Error: 400 Bad Request with error details
- Server Error: 500 Internal Server Error with generic error response

**Error Responses**:
```json
{
  "response": "I can only answer questions about the book content. Please ask a question related to the book.",
  "sources": []
}
```

## State Transitions

### Chat Session Flow
1. **Request Received**: ChatRequest is validated
2. **Context Determination**: System determines whether to use selected_text or Qdrant retrieval
3. **Context Retrieval**: Either uses selected_text directly or retrieves from Qdrant
4. **Agent Processing**: AI agent processes the user message with determined context
5. **Response Formatting**: Response is formatted for ChatKit compatibility
6. **Response Sent**: ChatResponse is returned to the client

## Relationships

### Entity Relationships
- ChatRequest → ChatContext (1:1 transformation)
- ChatContext → ChatResponse (1:1 transformation via AI processing)
- ChatRequest → ChatResponse (1:1 via processing pipeline)

## Constraints

### Business Constraints
- If `selected_text` exists in ChatRequest, vector retrieval from Qdrant must be bypassed
- All responses must be grounded in book content only
- Off-topic queries must receive polite refusal without HTTP errors
- Response time should be under 5 seconds for 95% of requests

### Technical Constraints
- Message length limited to 2000 characters
- Context window must accommodate both user message and retrieved content
- Responses must be compatible with ChatKit message format
- Error handling must be graceful and user-friendly

## Validation Rules Summary

1. **Input Validation**:
   - Message cannot be empty
   - Message length must be reasonable
   - Selected text, if provided, should be relevant

2. **Context Validation**:
   - Effective context must be properly formed
   - Selected text takes precedence over retrieved chunks
   - Context must be sufficient for AI processing

3. **Output Validation**:
   - Response must be non-empty when successful
   - Sources must reference actual book content
   - Error responses must be polite and helpful