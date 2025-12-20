import pytest
from unittest.mock import Mock, MagicMock
from src.retrieval.client import QdrantRetrievalClient
from src.retrieval.config import RetrievalConfig
from src.retrieval.models import SearchQuery, RetrievalResult, RetrievalRequest, RetrievalResponse
from datetime import datetime

@pytest.fixture
def mock_qdrant_client():
    """Mock Qdrant client for testing."""
    client = Mock(spec=QdrantRetrievalClient)
    client.search.return_value = []
    client.health_check.return_value = True
    return client


@pytest.fixture
def sample_config():
    """Sample configuration for testing."""
    config = RetrievalConfig()
    return config


@pytest.fixture
def sample_search_query():
    """Sample search query for testing."""
    return SearchQuery(
        id="test-query-1",
        text="What is ROS 2 navigation?",
        type="general",
        created_at=datetime.now()
    )


@pytest.fixture
def sample_retrieval_result():
    """Sample retrieval result for testing."""
    return RetrievalResult(
        id="test-result-1",
        chunk_id="chunk-001",
        text="ROS 2 navigation is a system for autonomous robot movement...",
        similarity_score=0.85,
        metadata={"module": "module4", "chapter": "ch7", "glossary": False},
        position=1
    )


@pytest.fixture
def sample_retrieval_request(sample_search_query):
    """Sample retrieval request for testing."""
    return RetrievalRequest(
        query=sample_search_query,
        top_k=10,
        include_metadata=True
    )


@pytest.fixture
def sample_retrieval_response(sample_retrieval_result):
    """Sample retrieval response for testing."""
    return RetrievalResponse(
        query_id="test-query-1",
        results=[sample_retrieval_result],
        total_results=1,
        execution_time_ms=125.5,
        timestamp=datetime.now()
    )


@pytest.fixture
def retrieval_test_data():
    """Sample test data for retrieval operations."""
    return {
        "queries": [
            "What is ROS 2?",
            "How does navigation work in robotics?",
            "Explain PID controllers",
            "What is computer vision in robotics?",
            "How do sensors work in ROS?",
        ],
        "expected_modules": [
            "module-1-ros",
            "module-4-advanced-integration",
            "module-3-nvidia-isaac",
            "module-2-simulation",
            "module-1-ros"
        ]
    }