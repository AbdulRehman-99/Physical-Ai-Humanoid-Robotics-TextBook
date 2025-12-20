from dataclasses import dataclass
from typing import Dict, List, Optional
from datetime import datetime

@dataclass
class SearchQuery:
    """
    Represents a search query with its parameters.
    """
    id: str
    text: str
    type: str  # 'general' or 'selected-text'
    created_at: datetime
    metadata_filters: Optional[Dict] = None

    def __post_init__(self):
        if self.type not in ['general', 'selected-text']:
            raise ValueError(f"Query type must be 'general' or 'selected-text', got '{self.type}'")


@dataclass
class RetrievalResult:
    """
    Represents a single retrieval result.
    """
    id: str
    chunk_id: str
    text: str
    similarity_score: float  # cosine similarity score between 0-1
    metadata: Dict
    position: int  # position in top-k results


@dataclass
class RetrievalRequest:
    """
    Represents a retrieval request with parameters.
    """
    query: SearchQuery
    top_k: int = 10
    include_metadata: bool = True
    exact_match_first: bool = False  # for selected-text approach


@dataclass
class RetrievalResponse:
    """
    Represents the response from a retrieval operation.
    """
    query_id: str
    results: List[RetrievalResult]
    total_results: int
    execution_time_ms: float
    timestamp: datetime


@dataclass
class PerformanceMetrics:
    """
    Represents performance metrics for a retrieval operation.
    """
    request_id: str
    latency_ms: float
    throughput_qps: float
    stability_score: float
    timestamp: datetime


@dataclass
class ValidationResult:
    """
    Represents validation result for a retrieval.
    """
    result_id: str
    relevance_score: float  # human-validated relevance 0-1
    metadata_accuracy: bool
    validation_notes: Optional[str] = None
    validated_by: Optional[str] = None
    validated_at: Optional[datetime] = None