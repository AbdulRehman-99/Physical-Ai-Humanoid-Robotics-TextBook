"""Module for semantic search functionality to support RAG chatbot queries."""

from typing import List, Dict, Any, Optional
from src.ingestion.vector_store import VectorStore
from src.ingestion.models import DocumentChunk
from src.config.settings import settings
import logging
import time
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class SearchResult:
    """Represents a search result with relevance score and metadata."""
    chunk_id: str
    text_content: str
    score: float
    metadata: Dict[str, Any]
    module: str
    chapter: str
    section: str
    source_file_path: str


class SearchInterface:
    """Class to handle semantic search queries against the vector database."""

    def __init__(self):
        """Initialize the search interface with vector store connection."""
        self.vector_store = VectorStore()

    def search(self, query: str, top_k: int = 5, min_score: float = 0.0) -> List[SearchResult]:
        """
        Perform semantic search against the vector database.

        Args:
            query: The search query text
            top_k: Number of top results to return (default: 5)
            min_score: Minimum relevance score threshold (default: 0.0)

        Returns:
            List of SearchResult objects sorted by relevance score
        """
        start_time = time.time()
        logger.info(f"Starting semantic search for query: '{query[:50]}...'")

        try:
            # Generate embedding for the query
            # Note: We'll need to use the embedder to generate the query embedding
            from src.ingestion.embedder import Embedder
            embedder = Embedder()
            query_embedding = embedder.embed_single_text(query)

            # Search in the vector store
            search_results = self.vector_store.search_vectors(
                query_vector=query_embedding,
                top_k=top_k,
                min_score=min_score
            )

            # Convert results to SearchResult objects
            formatted_results = []
            for result in search_results:
                search_result = SearchResult(
                    chunk_id=result.get('id', ''),
                    text_content=result.get('payload', {}).get('text_content', ''),
                    score=result.get('score', 0.0),
                    metadata=result.get('payload', {}),
                    module=result.get('payload', {}).get('module', ''),
                    chapter=result.get('payload', {}).get('chapter', ''),
                    section=result.get('payload', {}).get('section', ''),
                    source_file_path=result.get('payload', {}).get('source_file_path', '')
                )
                formatted_results.append(search_result)

            # Sort results by score in descending order
            formatted_results.sort(key=lambda x: x.score, reverse=True)

            duration = time.time() - start_time
            logger.info(f"Search completed in {duration:.3f}s, found {len(formatted_results)} results")

            return formatted_results

        except Exception as e:
            logger.error(f"Error during semantic search: {e}")
            logger.exception("Full traceback:")
            raise e

    def search_with_filters(self, query: str, filters: Optional[Dict[str, Any]] = None,
                           top_k: int = 5, min_score: float = 0.0) -> List[SearchResult]:
        """
        Perform semantic search with metadata filters.

        Args:
            query: The search query text
            filters: Dictionary of metadata filters (e.g., {'module': 'module1'})
            top_k: Number of top results to return (default: 5)
            min_score: Minimum relevance score threshold (default: 0.0)

        Returns:
            List of SearchResult objects sorted by relevance score
        """
        start_time = time.time()
        logger.info(f"Starting filtered semantic search for query: '{query[:50]}...' with filters: {filters}")

        try:
            # Generate embedding for the query
            from src.ingestion.embedder import Embedder
            embedder = Embedder()
            query_embedding = embedder.embed_single_text(query)

            # Search in the vector store with filters
            search_results = self.vector_store.search_vectors(
                query_vector=query_embedding,
                top_k=top_k,
                min_score=min_score,
                filters=filters
            )

            # Convert results to SearchResult objects
            formatted_results = []
            for result in search_results:
                search_result = SearchResult(
                    chunk_id=result.get('id', ''),
                    text_content=result.get('payload', {}).get('text_content', ''),
                    score=result.get('score', 0.0),
                    metadata=result.get('payload', {}),
                    module=result.get('payload', {}).get('module', ''),
                    chapter=result.get('payload', {}).get('chapter', ''),
                    section=result.get('payload', {}).get('section', ''),
                    source_file_path=result.get('payload', {}).get('source_file_path', '')
                )
                formatted_results.append(search_result)

            # Sort results by score in descending order
            formatted_results.sort(key=lambda x: x.score, reverse=True)

            duration = time.time() - start_time
            logger.info(f"Filtered search completed in {duration:.3f}s, found {len(formatted_results)} results")

            return formatted_results

        except Exception as e:
            logger.error(f"Error during filtered semantic search: {e}")
            logger.exception("Full traceback:")
            raise e

    def format_search_results(self, results: List[SearchResult],
                             include_citations: bool = True,
                             max_content_length: Optional[int] = None) -> str:
        """
        Format search results for display or use by the RAG chatbot.

        Args:
            results: List of SearchResult objects
            include_citations: Whether to include citations in the output
            max_content_length: Maximum length of content to include (None for no limit)

        Returns:
            Formatted string with search results
        """
        if not results:
            return "No relevant results found."

        formatted_output = []

        for i, result in enumerate(results, 1):
            # Truncate content if needed
            content = result.text_content
            if max_content_length and len(content) > max_content_length:
                content = content[:max_content_length] + "..."

            # Format the result
            result_text = f"Result {i} (Score: {result.score:.3f}):\n{content}\n"

            if include_citations:
                result_text += f"Source: Module '{result.module}', Chapter '{result.chapter}', Section '{result.section}'\n"
                result_text += f"File: {result.source_file_path}\n"

            result_text += "-" * 50 + "\n"
            formatted_output.append(result_text)

        return "\n".join(formatted_output)

    def get_response_time_monitoring(self) -> Dict[str, Any]:
        """
        Get monitoring information about search performance.

        Returns:
            Dictionary with performance metrics
        """
        # This would typically connect to monitoring infrastructure
        # For now, we'll return placeholder values
        return {
            "avg_response_time": 0.0,  # Average response time in seconds
            "last_response_time": 0.0,  # Last response time in seconds
            "total_queries": 0,  # Total number of queries processed
            "success_rate": 1.0,  # Success rate of queries
            "error_rate": 0.0,  # Error rate of queries
        }

    def validate_search_query(self, query: str) -> List[str]:
        """
        Validate a search query for quality and safety.

        Args:
            query: The search query to validate

        Returns:
            List of issues found with the query (empty list if valid)
        """
        issues = []

        if not query or not query.strip():
            issues.append("Query is empty or contains only whitespace")

        if len(query.strip()) < 3:
            issues.append("Query is too short (minimum 3 characters)")

        if len(query) > 500:  # Arbitrary limit
            issues.append("Query is too long (maximum 500 characters)")

        # Check for potential harmful content (simple check)
        harmful_keywords = ["DROP", "DELETE", "ALTER", "EXEC", "SCRIPT"]
        if any(keyword in query.upper() for keyword in harmful_keywords):
            issues.append("Query contains potentially harmful keywords")

        return issues


def create_search_engine():
    """
    Create a search engine instance that can be used by the RAG chatbot.

    Returns:
        A SearchInterface instance
    """
    return SearchInterface()