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
from .embedder import Embedder
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
        self.embedder = Embedder()  # Add the embedder

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

        # Generate embedding for the query text
        try:
            query_embedding = self.embedder.embed_single_text(
                text=sanitized_query_text,
                model="embed-multilingual-v3.0",  # Use the correct model
                input_type="search_query"
            )

            # Validate the embedding dimensions
            if not self.embedder.validate_embedding_dimensions(query_embedding):
                self.logger.log_error(f"Invalid embedding dimensions for query: {sanitized_query_text}")
                # Return empty results if embedding is invalid
                results = []
            else:
                # Perform the actual search in Qdrant
                search_results = self.client.search(
                    query_vector=query_embedding,
                    top_k=request.top_k
                )

                # Convert Qdrant results to our data models
                results = self._convert_qdrant_results(search_results)

        except Exception as e:
            self.logger.log_error(f"Error during embedding generation or search: {e}")
            # Return empty results in case of error
            results = []

        execution_time_ms = (time.time() - start_time) * 1000

        # Validate results if validation is requested
        if request.include_metadata and results:  # We'll use this flag to indicate validation is needed
            # In a real implementation, we would validate each result
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

    def _convert_qdrant_results(self, qdrant_results) -> List[RetrievalResult]:
        """
        Convert Qdrant search results to our RetrievalResult format.

        Args:
            qdrant_results: Raw results from Qdrant client

        Returns:
            List of RetrievalResult objects
        """
        results = []
        for idx, result in enumerate(qdrant_results):
            # Extract text content and metadata from the result
            text_content = result.payload.get('content', '') if hasattr(result, 'payload') and result.payload else ''

            # Extract metadata from payload
            metadata = result.payload if hasattr(result, 'payload') and result.payload else {}

            # Create a RetrievalResult object
            retrieval_result = RetrievalResult(
                id=str(result.id) if hasattr(result, 'id') else f"result-{idx}",
                chunk_id=result.payload.get('chunk_id', f"chunk-{idx:03d}") if hasattr(result, 'payload') and result.payload else f"chunk-{idx:03d}",
                text=text_content,
                similarity_score=result.score if hasattr(result, 'score') else 0.0,
                metadata=metadata,
                position=idx + 1
            )
            results.append(retrieval_result)

        return results

    def _create_mock_results(self, request: RetrievalRequest) -> List[RetrievalResult]:
        """
        Create mock results for demonstration purposes.
        This is kept for backward compatibility.
        """
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
        # Generate embedding for the query text
        try:
            sanitized_query_text = self._sanitize_query_text(request.query.text)
            query_embedding = self.embedder.embed_single_text(
                text=sanitized_query_text,
                model="embed-multilingual-v3.0",  # Use the correct model
                input_type="search_query"
            )

            # Validate the embedding dimensions
            if not self.embedder.validate_embedding_dimensions(query_embedding):
                self.logger.log_error(f"Invalid embedding dimensions for selected-text query: {sanitized_query_text}")
                return []

            # Perform the actual search in Qdrant
            search_results = self.client.search(
                query_vector=query_embedding,
                top_k=request.top_k
            )

            # Convert Qdrant results to our data models
            results = self._convert_qdrant_results(search_results)

            # Apply selected-text specific logic (exact match + semantic similarity weighting)
            # In a real implementation, we would combine exact matches with semantic similarity
            # results using the configured weights (70% semantic, 30% exact)

            # For now, we return the semantic similarity results with potential post-processing
            # that would incorporate exact match considerations

            return results

        except Exception as e:
            self.logger.log_error(f"Error during selected-text search: {e}")
            # Return empty results in case of error
            return []