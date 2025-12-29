"""
Initialization module for the retrieval package.
Provides compatibility layer for Agent interface.
"""

from .search import SemanticSearch
from .models import SearchQuery, RetrievalRequest
from .config import RetrievalConfig
from datetime import datetime

# Global retrieval service instance
_retrieval_service = None


def get_retrieval_service():
    """
    Get the retrieval service instance for compatibility with Agent interface.
    """
    global _retrieval_service
    if _retrieval_service is None:
        _retrieval_service = RetrievalServiceWrapper()
    return _retrieval_service


class RetrievalServiceWrapper:
    """
    Wrapper class to provide the same interface as expected by the Agent.
    """

    def __init__(self):
        self.search_service = SemanticSearch()
        self.config = RetrievalConfig()

    def get_book_context(self, query: str, top_k: int = None):
        """
        Retrieve book context based on a query for compatibility with Agent interface.

        Args:
            query: The query string to search for
            top_k: Number of top results to return (optional)

        Returns:
            String containing the context from the book
        """
        if top_k is None:
            top_k = getattr(self.config, 'top_k', 5)

        # Create a search query
        search_query = SearchQuery(
            id="agent-query",
            text=query,
            type="general",
            created_at=datetime.now()
        )

        # Create a retrieval request
        request = RetrievalRequest(
            query=search_query,
            top_k=top_k,
            include_metadata=True
        )

        # Perform the search
        response = self.search_service.search(request)

        # Extract content from results
        context_chunks = []
        for result in response.results:
            if result.text and result.text.strip():
                context_chunks.append(result.text)

        return " ".join(context_chunks)

    def search_similar(self, query: str, top_k: int = None):
        """
        Search for similar content for compatibility with Agent interface.

        Args:
            query: The query string to search for
            top_k: Number of top results to return (optional)

        Returns:
            List of similar content results
        """
        if top_k is None:
            top_k = getattr(self.config, 'top_k', 5)

        # Create a search query
        search_query = SearchQuery(
            id="similar-query",
            text=query,
            type="general",
            created_at=datetime.now()
        )

        # Create a retrieval request
        request = RetrievalRequest(
            query=search_query,
            top_k=top_k,
            include_metadata=True
        )

        # Perform the search
        response = self.search_service.search(request)

        # Return the results in a compatible format
        results = []
        for result in response.results:
            results.append({
                'content': result.text,
                'score': result.similarity_score,
                'metadata': result.metadata
            })

        return results