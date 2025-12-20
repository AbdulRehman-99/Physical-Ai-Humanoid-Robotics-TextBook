# Research Notes: Book Ingestion & Vectorization

## Decision: Project Structure
**Rationale**: Following user requirements to create a Backend folder with uv package management and a single main file (ingest_book.py) for the ingestion functionality.
**Alternatives considered**:
- Using existing frontend structure vs. creating separate backend
- Multiple files vs. single main file approach
- Different package managers (pip vs. uv)

## Decision: Technology Stack
**Rationale**: Python was specified by the user as the main language, with Cohere for embeddings and Qdrant for vector storage. UV was specified as the package manager.
**Alternatives considered**:
- Different programming languages (JavaScript/TypeScript, Go)
- Different embedding providers (OpenAI, Hugging Face, Google)
- Different vector databases (Pinecone, Weaviate, Chroma)

## Decision: Document Processing Approach
**Rationale**: Processing all .md files from Frontend/docs directory as specified, with semantic chunking to preserve context and overlap to maintain coherence across chunks.
**Alternatives considered**:
- Processing different file formats (PDF, DOCX, HTML)
- Different chunking strategies (fixed-size vs. semantic)
- Different overlap strategies (no overlap vs. configurable overlap)

## Decision: Metadata Preservation
**Rationale**: Preserving module, chapter, section, and book_version metadata as specified to maintain document context and enable proper search results.
**Alternatives considered**:
- Minimal metadata vs. comprehensive metadata
- Different metadata extraction methods

## Decision: Error Handling Strategy
**Rationale**: Implementing graceful error handling to continue processing when individual files fail, as specified in the feature requirements.
**Alternatives considered**:
- Stop-on-error vs. continue-on-error approaches
- Different logging strategies for errors

## Decision: Environment Configuration
**Rationale**: Using .env file for API keys as specified (COHERE_API_KEY, QDRANT_API_KEY, QDRANT_URL) to keep credentials secure.
**Alternatives considered**:
- Different configuration management approaches (command-line args, config files, environment variables only)

## Decision: RAG Chatbot Integration
**Rationale**: The system must integrate with the Book RAG Chatbot as required by the project constitution, enabling semantic search capabilities for students.
**Alternatives considered**:
- Standalone vector storage vs. integrated RAG system
- Different API approaches for chatbot integration (REST vs. GraphQL vs. direct SDK)

## Decision: Performance Requirements
**Rationale**: Search response time of ≤2 seconds and ≥95% success rate for document processing are required for good user experience.
**Alternatives considered**:
- Different performance thresholds based on system capabilities
- Various retry and backoff strategies for API calls