# Quickstart: Retrieval Testing Scripts

## Prerequisites

- Python 3.11+
- Access to Qdrant vector database with existing book content collection
- pip for dependency management (no uv, FastAPI, or additional frameworks)

## Setup

1. **Install dependencies**:
   ```bash
   cd Backend
   pip install qdrant-client numpy pandas pytest python-dotenv
   ```

2. **Configure environment**:
   ```bash
   # Create .env file in Backend root
   QDRANT_HOST=localhost
   QDRANT_PORT=6333
   QDRANT_COLLECTION_NAME=book_content
   ```

3. **Create the retrieval module**:
   ```bash
   mkdir -p Backend/src/retrieval
   touch Backend/src/retrieval/__init__.py
   ```

## Basic Usage

1. **Run the main retrieval test script**:
   ```bash
   cd Backend
   python -m src.retrieval.run_tests
   ```

2. **Or execute individual components**:
   ```python
   from Backend.src.retrieval.client import QdrantRetrievalClient

   client = QdrantRetrievalClient()
   results = client.search(
       query_text="What is ROS 2 navigation?",
       top_k=10,
       include_metadata=True
   )
   ```

3. **Validate retrieval accuracy**:
   ```python
   from Backend.src.retrieval.validation import validate_relevance

   validation_result = validate_relevance(
       query="What is ROS 2 navigation?",
       retrieved_results=results,
       expected_section="module-4-advanced-integration/ch7-whisper-llm-vla-planning"
   )
   ```

4. **Measure performance**:
   ```python
   from Backend.src.retrieval.latency import measure_latency

   metrics = measure_latency(
       search_function=client.search,
       query_text="test query",
       iterations=100
   )
   ```

## Testing

Run the retrieval tests:
```bash
cd Backend
pytest tests/retrieval/ -v
```

## Configuration

The scripts support configurable parameters:
- `TOP_K` - Number of results to return (default: 10)
- `SIMILARITY_THRESHOLD` - Minimum similarity score (default: 0.5)
- `MAX_LATENCY_MS` - Target maximum response time (default: 500)

## Clean Handoff to Step-3

The lightweight scripts are designed with a clean handoff to Step-3 implementation:
- All core functionality is modular and can be integrated into larger systems
- Data models and interfaces are compatible with future service implementations
- Logging and metrics are structured for easy integration into monitoring systems