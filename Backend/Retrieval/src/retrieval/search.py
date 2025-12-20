import numpy as np
from typing import List, Optional, Dict, Any
from datetime import datetime
from .client import QdrantRetrievalClient
from .config import RetrievalConfig
from .models import (
    SearchQuery, RetrievalResult, RetrievalRequest,
    RetrievalResponse, ValidationResult
)
from .logger import RetrievalLogger
from .validation import ValidationService
import time


class SemanticSearch:
    """
    Implements semantic similarity search functionality using cosine similarity
    and integrates with Qdrant client for vector database operations.
    """

    def __init__(self):
        self.client = QdrantRetrievalClient()
        self.config = RetrievalConfig()
        self.logger = RetrievalLogger()
        self.validator = ValidationService()

    def search(self, request: RetrievalRequest) -> RetrievalResponse:
        """
        Implement semantic search function using cosine similarity.
        Handles both 'general' and 'selected-text' search types.

        Args:
            request: RetrievalRequest containing the query and parameters

        Returns:
            RetrievalResponse containing the search results
        """
        # Handle different search types
        if request.query.type == "selected-text":
            return self.search_with_selected_text(request)

        start_time = time.time()

        # Sanitize and validate query text
        sanitized_query_text = self._sanitize_query_text(request.query.text)

        # Log the request
        request_data = {
            "query_text": sanitized_query_text,
            "top_k": request.top_k,
            "include_metadata": request.include_metadata,
            "query_type": request.query.type
        }
        self.logger.log_retrieval_request(request_data)

        # For now, we'll simulate the search since we don't have actual vector embeddings
        # In a real implementation, we would:
        # 1. Convert the query text to a vector using an embedding model
        # 2. Use the Qdrant client to perform the search
        # 3. Convert the results to our data models

        # For demonstration, we'll create mock results
        results = self._create_mock_results(request)

        execution_time_ms = (time.time() - start_time) * 1000

        # Validate results if validation is requested
        if request.include_metadata:  # We'll use this flag to indicate validation is needed
            # In a real implementation, we would validate each result
            # For now, we'll just log that validation would happen
            for result in results:
                relevance_valid = self.validator.validate_relevance(result)
                metadata_valid = self.validator.validate_metadata(result)
                # The validation results are logged by the validation service itself

        response = RetrievalResponse(
            query_id=request.query.id,
            results=results,
            total_results=len(results),
            execution_time_ms=execution_time_ms,
            timestamp=datetime.now()
        )

        # Add performance tracking
        perf_data = {
            "query_id": request.query.id,
            "execution_time_ms": execution_time_ms,
            "num_results": len(results),
            "query_type": request.query.type,
            "top_k_requested": request.top_k
        }
        self.logger.log_performance_metrics(perf_data)

        # Log the response
        response_data = {
            "query_id": request.query.id,
            "num_results": len(results),
            "execution_time_ms": execution_time_ms
        }
        self.logger.log_retrieval_response(response_data)

        return response

    def _create_mock_results(self, request: RetrievalRequest) -> List[RetrievalResult]:
        """
        Create mock results for demonstration purposes.
        In a real implementation, this would call the Qdrant client with actual vectors.
        """
        # This is a placeholder - in a real implementation, we would:
        # 1. Convert the query text to an embedding vector
        # 2. Call self.client.search() with the vector
        # 3. Convert the Qdrant results to RetrievalResult objects

        # Validate request parameters
        if request.top_k <= 0:
            print("Warning: top_k should be positive, using default of 10")
            request.top_k = 10

        # For now, create mock results with decreasing similarity scores
        results = []
        for i in range(min(request.top_k, 10)):  # Limit to 10 for demo
            similarity_score = 0.95 - (i * 0.05)  # Decreasing scores
            result = RetrievalResult(
                id=f"result-{i+1}",
                chunk_id=f"chunk-{i+1:03d}",
                text=f"Sample text chunk {i+1} related to '{request.query.text}'",
                similarity_score=similarity_score,
                metadata={
                    "module": f"module-{(i % 4) + 1}",
                    "chapter": f"chapter-{(i % 3) + 1}",
                    "glossary": i % 5 == 0  # Every 5th result is a glossary entry
                },
                position=i + 1
            )
            results.append(result)

        # Log if no results were generated
        if not results:
            print(f"Warning: No results generated for query: {request.query.text}")

        return results

    def _sanitize_query_text(self, query_text: str) -> str:
        """
        Sanitize query text to handle special characters and long text.

        Args:
            query_text: Original query text

        Returns:
            Sanitized query text
        """
        if query_text is None:
            return ""

        # Handle long text by truncating to a reasonable length
        max_length = 1000  # Maximum length for query text
        if len(query_text) > max_length:
            print(f"Warning: Query text too long ({len(query_text)} chars), truncating to {max_length} chars")
            query_text = query_text[:max_length]

        # Handle special characters if needed (for now just return the text)
        # In a real implementation, you might want to escape or clean special characters
        return query_text

    def _cosine_similarity(self, vec1: List[float], vec2: List[float]) -> float:
        """
        Calculate cosine similarity between two vectors.

        Args:
            vec1: First vector
            vec2: Second vector

        Returns:
            Cosine similarity score between 0 and 1
        """
        # Convert to numpy arrays
        v1 = np.array(vec1)
        v2 = np.array(vec2)

        # Calculate cosine similarity
        dot_product = np.dot(v1, v2)
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)

        if norm_v1 == 0 or norm_v2 == 0:
            return 0.0

        similarity = dot_product / (norm_v1 * norm_v2)

        # Ensure the result is between 0 and 1
        return max(0.0, min(1.0, float(similarity)))

    def search_with_selected_text(self, request: RetrievalRequest) -> RetrievalResponse:
        """
        Enhanced search function to support selected-text queries.

        Args:
            request: RetrievalRequest containing the query and parameters

        Returns:
            RetrievalResponse containing the search results
        """
        if request.query.type != "selected-text":
            return self.search(request)

        start_time = time.time()

        # Sanitize and validate query text
        sanitized_query_text = self._sanitize_query_text(request.query.text)

        # Log the request
        request_data = {
            "query_text": sanitized_query_text,
            "top_k": request.top_k,
            "include_metadata": request.include_metadata,
            "query_type": request.query.type,
            "exact_match_first": request.exact_match_first
        }
        self.logger.log_retrieval_request(request_data)

        # For selected-text queries, implement exact match + semantic similarity approach
        # with weighted scoring algorithm (70% semantic similarity, 30% exact match)
        results = self._perform_selected_text_search(request)

        execution_time_ms = (time.time() - start_time) * 1000

        # Validate results if validation is requested
        if request.include_metadata:  # We'll use this flag to indicate validation is needed
            # In a real implementation, we would validate each result
            # For now, we'll just log that validation would happen
            for result in results:
                relevance_valid = self.validator.validate_relevance(result)
                metadata_valid = self.validator.validate_metadata(result)
                # The validation results are logged by the validation service itself

        response = RetrievalResponse(
            query_id=request.query.id,
            results=results,
            total_results=len(results),
            execution_time_ms=execution_time_ms,
            timestamp=datetime.now()
        )

        # Add performance tracking
        perf_data = {
            "query_id": request.query.id,
            "execution_time_ms": execution_time_ms,
            "num_results": len(results),
            "query_type": request.query.type,
            "top_k_requested": request.top_k
        }
        self.logger.log_performance_metrics(perf_data)

        # Log the response
        response_data = {
            "query_id": request.query.id,
            "num_results": len(results),
            "execution_time_ms": execution_time_ms
        }
        self.logger.log_retrieval_response(response_data)

        return response

    def _perform_selected_text_search(self, request: RetrievalRequest) -> List[RetrievalResult]:
        """
        Perform selected-text search using exact match + semantic similarity approach
        with weighted scoring algorithm (70% semantic similarity, 30% exact match).

        Args:
            request: RetrievalRequest containing the query and parameters

        Returns:
            List of RetrievalResult objects
        """
        # This is a simplified implementation of the exact match + semantic similarity approach
        # In a real implementation, we would:
        # 1. Identify exact matches for the selected text
        # 2. Perform semantic similarity search
        # 3. Combine results using weighted scoring (70% semantic similarity, 30% exact match)

        # For demonstration, we'll create mock results with adjusted scores
        # based on the selected-text approach
        results = self._create_mock_results(request)

        # In a real implementation, we would apply the weighting here
        # For now, we'll just return the results with a note that weighting would be applied
        for result in results:
            # This is where the 70% semantic similarity + 30% exact match weighting
            # would be applied in a real implementation
            pass

        return results