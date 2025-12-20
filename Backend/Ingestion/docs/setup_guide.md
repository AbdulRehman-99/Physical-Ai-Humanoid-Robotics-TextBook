# Setup Guide for Book Ingestion System

This guide will help you set up the book ingestion system that processes markdown files, generates embeddings using Cohere, and stores them in Qdrant for semantic search.

## Prerequisites

- Python 3.8 or higher
- pip package manager
- Git (optional, for cloning the repository)

## Environment Setup

### 1. Clone the Repository (if applicable)

```bash
git clone <repository-url>
cd <repository-directory>
```

### 2. Navigate to Backend Directory

```bash
cd Backend
```

### 3. Install uv Package Manager

If you don't have `uv` installed:

```bash
pip install uv
```

### 4. Install Dependencies

```bash
uv pip install -r requirements.txt
```

Or if using the pyproject.toml approach:

```bash
uv sync
```

## Configuration

### 1. Create Environment File

Create a `.env` file in the Backend root directory with the following content:

```env
# Cohere API Configuration
COHERE_API_KEY=your_cohere_api_key_here
COHERE_RETRY_ATTEMPTS=5
COHERE_BASE_DELAY=1.0
COHERE_EMBEDDING_DIMENSIONS=1024

# Qdrant Configuration
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_COLLECTION_NAME=book_embeddings

# Processing Configuration
CHUNK_SIZE=512
CHUNK_OVERLAP=100
MAX_DOCUMENT_SIZE=10485760
LOG_LEVEL=INFO
```

### 2. Obtain API Keys

- **Cohere API Key**: Sign up at [Cohere](https://cohere.com/) and create an API key
- **Qdrant API Key & URL**: Set up a Qdrant instance (cloud or local) and obtain the credentials

## Running the Ingestion Pipeline

### 1. Basic Usage

```bash
cd Backend
python ingest_book.py
```

### 2. With Custom Parameters

```bash
python ingest_book.py --source-path "../Frontend/docs" --chunk-size 1024 --chunk-overlap 200 --log-level DEBUG
```

### 3. Available Options

```bash
python ingest_book.py --help
```

Common options:
- `--source-path`: Path to markdown files (default: `Frontend/docs`)
- `--target-collection`: Qdrant collection name (default: from settings)
- `--chunk-size`: Size of text chunks (default: from settings)
- `--chunk-overlap`: Overlap between chunks (default: from settings)
- `--validate-quality`: Validate embedding quality (default: True)
- `--log-level`: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)

## API Endpoints

The system provides REST API endpoints for RAG chatbot integration:

### Health Check
```
GET /health
```

### Semantic Search
```
POST /search
Content-Type: application/json

{
  "query": "your search query",
  "top_k": 5,
  "min_score": 0.5,
  "filters": {"module": "module1"},
  "include_citations": true
}
```

### Formatted Search
```
POST /search/formatted
Content-Type: application/json

{
  "query": "your search query",
  "top_k": 3,
  "include_citations": true,
  "max_content_length": 500
}
```

## Running Tests

### Run All Tests

```bash
cd Backend
python -m pytest tests/ -v
```

### Run Specific Test Suite

```bash
python -m pytest tests/test_comprehensive.py -v
```

## Docker Deployment (Optional)

If Docker is preferred for deployment:

### 1. Build the Image

```bash
docker build -t book-ingestion-system .
```

### 2. Run with Environment Variables

```bash
docker run -e COHERE_API_KEY=your_key -e QDRANT_API_KEY=your_key -e QDRANT_URL=your_url book-ingestion-system
```

## Troubleshooting

### Common Issues

1. **API Key Errors**: Verify that your API keys are correct and have the necessary permissions
2. **Connection Errors**: Check that your Qdrant URL is accessible
3. **Memory Issues**: For large documents, consider processing in batches or increasing system memory
4. **Rate Limiting**: The system implements exponential backoff for API calls, but ensure you're within usage limits

### Logging

Check the logs for detailed error information. By default, logs are written to the console. You can increase the log level to DEBUG for more detailed information.

## Performance Considerations

- Processing time depends on the number and size of documents
- Embedding generation is the most time-consuming step
- Consider running the ingestion during off-peak hours for large document sets
- Monitor your Cohere API usage to stay within rate limits

## Security

- Never commit API keys to version control
- Use environment variables or secure key management systems
- Ensure the Qdrant instance is properly secured
- Validate all inputs when using the API endpoints