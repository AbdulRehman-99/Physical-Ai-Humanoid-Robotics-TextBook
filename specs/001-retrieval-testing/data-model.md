# Data Model: Retrieval Testing Scripts

## Core Entities

### SearchQuery
- **fields**:
  - id: string (unique identifier)
  - text: string (the query text)
  - type: enum ['general', 'selected-text'] (query type)
  - created_at: datetime (timestamp)
  - metadata_filters: dict (optional filters for metadata)

### RetrievalResult
- **fields**:
  - id: string (unique identifier)
  - chunk_id: string (identifier for the text chunk)
  - text: string (the retrieved text content)
  - similarity_score: float (cosine similarity score between 0-1)
  - metadata: dict (all metadata from Frontend/docs)
  - position: int (position in top-k results)

### RetrievalRequest
- **fields**:
  - query: SearchQuery (the input query)
  - top_k: int (number of results to return, default 10)
  - include_metadata: bool (whether to include metadata)
  - exact_match_first: bool (for selected-text approach)

### RetrievalResponse
- **fields**:
  - query_id: string (reference to the original query)
  - results: list[RetrievalResult] (top-k results)
  - total_results: int (total number of matching results)
  - execution_time_ms: float (time taken for retrieval)
  - timestamp: datetime (when the query was executed)

### PerformanceMetrics
- **fields**:
  - request_id: string (reference to the request)
  - latency_ms: float (response time)
  - throughput_qps: float (queries per second)
  - stability_score: float (consistency measure)
  - timestamp: datetime

### ValidationResult
- **fields**:
  - result_id: string (reference to the retrieval result)
  - relevance_score: float (human-validated relevance 0-1)
  - metadata_accuracy: bool (whether metadata is correct)
  - validation_notes: string (optional notes)
  - validated_by: string (validator identifier)
  - validated_at: datetime

## Relationships

- One `SearchQuery` produces many `RetrievalResult` items in a `RetrievalResponse`
- One `RetrievalRequest` generates one `RetrievalResponse`
- One `RetrievalResponse` has many `PerformanceMetrics` records
- One `RetrievalResult` has zero or one `ValidationResult`

## Script Data Flow

The lightweight retrieval testing scripts will use these data models in a simple, sequential flow:
1. Load test queries from `test_queries.py`
2. Execute searches using the Qdrant client
3. Validate results and measure performance
4. Log results to JSON files for analysis