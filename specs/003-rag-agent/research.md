# Research Document: RAG Agent Implementation

## Decision: Python Package Management with uv
**Rationale**: Using uv for Python 3.13 project initialization provides fast dependency resolution and management. uv is a drop-in replacement for pip and pip-tools, with significantly faster performance.

**Alternatives considered**:
- pip + venv: Standard but slower
- Poetry: More complex for this project scope
- Conda: Overkill for this project

## Decision: LLM Adapter Implementation
**Rationale**: Using LiteLLM as an adapter to connect OpenAI Agents SDK to Google Gemini provides compatibility while maintaining the required OpenAI Agents SDK v0.6 interface. LiteLLM handles API translation between OpenAI and Gemini formats.

**Alternatives considered**:
- Custom adapter: More control but higher development time
- Direct Gemini SDK: Would require abandoning OpenAI Agents SDK requirement
- OpenAI models: Doesn't meet requirement to use Gemini

## Decision: Vector Database Integration
**Rationale**: Qdrant with Cohere embeddings provides a robust, scalable solution for the RAG system. Cohere embeddings are known for quality and Qdrant provides efficient similarity search.

**Alternatives considered**:
- Pinecone: Proprietary, more expensive
- Weaviate: Good alternative but Cohere+Qdrant integration is well-documented
- FAISS: Requires more manual setup for production use

## Decision: Context Switching Logic
**Rationale**: Implementing explicit context switching based on selected_text parameter ensures strict adherence to the specification requirements. When selected_text is provided, only that text is used; otherwise, vector retrieval is used.

**Alternatives considered**:
- Combining contexts: Would violate the requirement that selected text should override vector retrieval
- Different parameter names: selected_text is clear and follows common patterns

## Decision: Error Handling and Guardrails
**Rationale**: Implementing comprehensive error handling ensures the system gracefully handles edge cases like vector database unavailability, oversized selected text, and off-topic queries as specified in the requirements.

**Alternatives considered**:
- Less comprehensive error handling: Would not meet the specification requirements
- Different error response formats: The chosen approach aligns with REST API best practices