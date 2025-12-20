# Usage Guide for Book Ingestion System

This guide provides detailed instructions on how to use the book ingestion system for processing educational content and enabling semantic search capabilities.

## Overview

The Book Ingestion System processes markdown files from the `Frontend/docs` directory, cleans and normalizes text, chunks it semantically with overlap, generates embeddings using Cohere, and stores them in Qdrant for semantic search. This enables a RAG (Retrieval Augmented Generation) chatbot to answer questions about the educational material.

## System Architecture

```
Markdown Files → Document Processor → Text Cleaner → Semantic Chunker → Embedder → Vector Store (Qdrant)
     ↓
API Endpoints ← Search Interface ← RAG Chatbot Integration
```

## Core Components

### 1. Document Processor
- Scans and processes markdown files from specified directories
- Extracts metadata (module, chapter, section, book_version) from file paths
- Handles frontmatter and validates document content

### 2. Text Cleaner
- Removes markdown formatting while preserving content
- Normalizes whitespace and line breaks
- Handles special characters and encoding issues
- Performs quality checks on cleaned text

### 3. Semantic Chunker
- Splits large documents into semantically coherent chunks
- Maintains configurable overlap between chunks (default: 100 tokens)
- Preserves sentence boundaries during chunking
- Validates chunk quality

### 4. Embedder
- Generates vector embeddings using Cohere API
- Implements exponential backoff for API rate limiting
- Validates embedding dimensions and quality

### 5. Vector Store
- Stores embeddings with metadata in Qdrant
- Provides semantic search capabilities
- Handles vector storage operations

## Command-Line Usage

### Basic Ingestion

```bash
cd Backend
python ingest_book.py
```

This processes all markdown files in `Frontend/docs` using default settings.

### Advanced Options

```bash
python ingest_book.py \
  --source-path "/path/to/markdown/files" \
  --chunk-size 1024 \
  --chunk-overlap 200 \
  --target-collection "my_collection" \
  --log-level DEBUG
```

### Configuration Options

- `--source-path`: Directory containing markdown files (default: `Frontend/docs`)
- `--target-collection`: Qdrant collection name (default: from .env)
- `--chunk-size`: Maximum tokens per chunk (default: 512)
- `--chunk-overlap`: Overlap tokens between chunks (default: 100)
- `--validate-quality`: Validate embeddings before storage (default: True)
- `--log-level`: Logging verbosity (DEBUG, INFO, WARNING, ERROR)

## API Usage

### Search Endpoint

The system provides REST API endpoints for semantic search:

```bash
curl -X POST http://localhost:5000/search \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS?",
    "top_k": 5,
    "min_score": 0.5,
    "filters": {"module": "module1"},
    "include_citations": true
  }'
```

### Formatted Search Endpoint

For direct integration with chatbots:

```bash
curl -X POST http://localhost:5000/search/formatted \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain humanoid robotics basics",
    "top_k": 3,
    "include_citations": true,
    "max_content_length": 500
  }'
```

### Response Format

Search responses include:
- Query text
- Search results with content, scores, and metadata
- Total results count
- Response time
- Search parameters used

## Configuration

### Environment Variables

Configure the system using environment variables in `.env`:

```env
# Cohere API
COHERE_API_KEY=your_api_key
COHERE_RETRY_ATTEMPTS=5
COHERE_BASE_DELAY=1.0
COHERE_EMBEDDING_DIMENSIONS=1024

# Qdrant
QDRANT_API_KEY=your_api_key
QDRANT_URL=your_url
QDRANT_COLLECTION_NAME=book_embeddings

# Processing
CHUNK_SIZE=512
CHUNK_OVERLAP=100
MAX_DOCUMENT_SIZE=10485760
LOG_LEVEL=INFO
```

### Processing Parameters

- **Chunk Size**: 256-1024 tokens (balance between context and efficiency)
- **Chunk Overlap**: 50-200 tokens (maintains context across boundaries)
- **Max Document Size**: 10MB limit (adjust based on system capacity)

## Integration with RAG Chatbot

The system is designed for seamless integration with RAG chatbots:

1. **Semantic Search**: Query the system to find relevant content
2. **Citation Support**: Results include source metadata for proper attribution
3. **Quality Filtering**: Results can be filtered by relevance score
4. **Metadata Filtering**: Search within specific modules/chapters

### Example Integration Code

```python
import requests

def query_knowledge_base(query: str, top_k: int = 3):
    response = requests.post(
        "http://localhost:5000/search/formatted",
        json={
            "query": query,
            "top_k": top_k,
            "include_citations": True
        }
    )
    return response.json()["formatted_results"]
```

## Performance Monitoring

The system includes built-in performance monitoring:

- **Processing Time**: Tracks duration of each pipeline step
- **Success Rates**: Monitors successful vs failed operations
- **Memory Usage**: Tracks memory consumption during processing
- **API Usage**: Monitors Cohere API calls and rate limits

## Error Handling

### Common Errors and Solutions

1. **API Rate Limits**: System implements exponential backoff; check usage limits
2. **Memory Issues**: Process large documents in batches; increase system memory
3. **Invalid Markdown**: Ensure files have valid markdown syntax
4. **Network Issues**: Verify API endpoint connectivity

### Logging

- Log level configured via `LOG_LEVEL` environment variable
- Detailed logs available in DEBUG mode
- Error logs include stack traces for debugging

## Best Practices

### For Optimal Performance

1. **Batch Processing**: Process multiple documents together when possible
2. **Chunk Size**: Use 512-1024 tokens for optimal balance of context and efficiency
3. **Overlap**: Maintain 100-200 token overlap to preserve context
4. **API Keys**: Securely manage API keys and monitor usage

### For Quality Results

1. **Content Quality**: Ensure source documents are well-structured and error-free
2. **Metadata Consistency**: Maintain consistent module/chapter/section naming
3. **Regular Updates**: Re-process content when source documents change
4. **Validation**: Use quality validation features to maintain content standards

## Maintenance

### Regular Tasks

1. **Monitor API Usage**: Check Cohere and Qdrant usage limits
2. **Review Logs**: Regularly check system logs for errors or warnings
3. **Update Content**: Re-run ingestion when educational content is updated
4. **Performance Review**: Monitor system performance and optimize as needed

### Backup and Recovery

- Regularly backup Qdrant collections
- Maintain copies of processed embeddings
- Document processing parameters for reproducibility