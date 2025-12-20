# Retrieval Testing Scripts Usage Guide

## Overview

The retrieval testing scripts provide comprehensive testing for semantic similarity search in the book content vector database. The system includes:

- Semantic similarity search functionality
- Validation of retrieval relevance and metadata
- Selected-text-only retrieval with exact match + semantic similarity
- Performance measurement and logging

## Prerequisites

- Python 3.11+
- Access to Qdrant vector database with existing book content collection
- pip for dependency management

## Setup

### 1. Install Dependencies

```bash
cd Backend
pip install -r requirements.txt
```

### 2. Configure Environment

Create a `.env` file in the Backend root:

```bash
QDRANT_HOST=localhost
QDRANT_PORT=6333
QDRANT_COLLECTION_NAME=book_content
TOP_K=10
SIMILARITY_THRESHOLD=0.5
MAX_LATENCY_MS=500
SEMANTIC_WEIGHT=0.7
EXACT_MATCH_WEIGHT=0.3
```

### 3. Create the Retrieval Module

The module structure is already created:

```
Backend/src/retrieval/
├── __init__.py
├── client.py          # Qdrant client setup
├── search.py          # Semantic similarity search implementation
├── validation.py      # Relevance and metadata validation
├── latency.py         # Performance measurement utilities
├── logger.py          # Logging for accuracy checks
├── config.py          # Configuration for k values and thresholds
├── run_tests.py       # Main script to execute retrieval tests
└── test_queries.py    # Sample book queries for testing
```

## Basic Usage

### Run the Main Retrieval Test Script

```bash
cd Backend
python -m src.retrieval.run_tests
```

### Run with Command-Line Options

```bash
# Run with specific queries
python -m src.retrieval.run_tests --queries "What is ROS?" "Explain navigation"

# Run with different top-k value
python -m src.retrieval.run_tests --top-k 20

# Run with verbose output
python -m src.retrieval.run_tests --verbose
```

### Use Individual Components

```python
from src.retrieval.search import SemanticSearch
from src.retrieval.models import SearchQuery, RetrievalRequest
from datetime import datetime

# Initialize the search service
search_service = SemanticSearch()

# Create a search query
query = SearchQuery(
    id="my-query-1",
    text="What is ROS 2 navigation?",
    type="general",  # or "selected-text"
    created_at=datetime.now()
)

# Create a retrieval request
request = RetrievalRequest(
    query=query,
    top_k=10,
    include_metadata=True
)

# Perform the search
response = search_service.search(request)

print(f"Retrieved {len(response.results)} results in {response.execution_time_ms:.2f}ms")
for result in response.results:
    print(f"- Score: {result.similarity_score:.3f}, Text: {result.text[:100]}...")
```

### Validate Retrieval Results

```python
from src.retrieval.validation import ValidationService

validator = ValidationService()

# Validate individual results
for result in response.results:
    is_relevant = validator.validate_relevance(result)
    has_valid_metadata = validator.validate_metadata(result)
    relevance_score = validator.validate_content_relevance("What is ROS 2 navigation?", result)

    print(f"Result {result.id}: Relevant={is_relevant}, Valid Metadata={has_valid_metadata}, Score={relevance_score:.3f}")
```

### Measure Performance

```python
from src.retrieval.latency import LatencyMeasurement

latency_service = LatencyMeasurement()

# Measure performance for a single query
perf_metrics = latency_service.add_performance_tracking_to_search(
    search_service.search,
    "performance test query",
    top_k=10
)

print(f"Execution time: {perf_metrics['execution_time_ms']:.2f}ms")

# Run comprehensive performance tests
perf_result = latency_service.measure_latency(
    lambda q, k=10: search_service.search(
        RetrievalRequest(
            query=SearchQuery(
                id="perf-test",
                text=q,
                type="general",
                created_at=datetime.now()
            ),
            top_k=k
        )
    ),
    "performance test query",
    iterations=100
)

print(f"Average latency: {perf_result.latency_ms:.2f}ms")
print(f"Throughput: {perf_result.throughput_qps:.2f} QPS")
print(f"Stability: {perf_result.stability_score:.2f}")
```

## Testing

Run the retrieval tests:

```bash
cd Backend
pytest tests/retrieval/ -v
```

## Configuration

The system supports the following configuration options via environment variables:

- `TOP_K`: Number of results to return (default: 10)
- `SIMILARITY_THRESHOLD`: Minimum similarity score (default: 0.5)
- `MAX_LATENCY_MS`: Target maximum response time (default: 500)
- `SEMANTIC_WEIGHT`: Weight for semantic similarity in selected-text search (default: 0.7)
- `EXACT_MATCH_WEIGHT`: Weight for exact match in selected-text search (default: 0.3)
- `QDRANT_HOST`: Qdrant server host (default: localhost)
- `QDRANT_PORT`: Qdrant server port (default: 6333)
- `QDRANT_COLLECTION_NAME`: Qdrant collection name (default: book_content)

## Data Models

The system uses the following data models:

- `SearchQuery`: Represents a search query with parameters
- `RetrievalResult`: Represents a single retrieval result
- `RetrievalRequest`: Represents a retrieval request with parameters
- `RetrievalResponse`: Represents the response from a retrieval operation
- `PerformanceMetrics`: Represents performance metrics for a retrieval operation
- `ValidationResult`: Represents validation result for a retrieval

## Logging

The system logs the following information:

- Retrieval requests and responses
- Performance metrics
- Validation results
- Individual chunks for accuracy analysis

Logs are stored in JSON format in `retrieval_logs.json` by default.

## Clean Handoff to Step-3

The lightweight scripts are designed with a clean handoff to Step-3 implementation:

- All core functionality is modular and can be integrated into larger systems
- Data models and interfaces are compatible with future service implementations
- Logging and metrics are structured for easy integration into monitoring systems

## Security Considerations

- Configuration values (like Qdrant credentials) should be stored securely
- Input validation is performed on all user-provided queries
- Query length is limited to prevent resource exhaustion