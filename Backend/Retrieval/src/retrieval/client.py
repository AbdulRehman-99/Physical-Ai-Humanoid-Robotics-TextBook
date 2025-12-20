import os
from typing import List, Dict, Optional
from qdrant_client import QdrantClient
from qdrant_client.http.models import models
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class QdrantRetrievalClient:
    """
    Qdrant client for retrieval operations on book content vector database.
    """

    def __init__(self):
        self.host = os.getenv("QDRANT_HOST", "localhost")
        self.port = int(os.getenv("QDRANT_PORT", "6333"))
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_content")

        # Initialize Qdrant client
        self.client = QdrantClient(
            host=self.host,
            port=self.port
        )

    def search(
        self,
        query_vector: List[float],
        top_k: int = 10,
        query_filter: Optional[models.Filter] = None
    ) -> List[models.ScoredPoint]:
        """
        Perform semantic similarity search in the Qdrant collection.

        Args:
            query_vector: Vector representation of the query text
            top_k: Number of top results to return
            query_filter: Optional filter to apply to the search

        Returns:
            List of ScoredPoint objects containing the search results
        """
        try:
            # Validate inputs
            if not query_vector:
                raise ValueError("Query vector cannot be empty")
            if top_k <= 0:
                raise ValueError("top_k must be a positive integer")
            if top_k > 100:  # Set a reasonable upper limit
                raise ValueError("top_k cannot exceed 100")

            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k,
                query_filter=query_filter
            )
            return results
        except ValueError as ve:
            # Handle validation errors
            print(f"Validation error during search: {ve}")
            return []
        except Exception as e:
            # Handle other errors
            print(f"Error performing search: {e}")
            return []

    def get_collection_info(self) -> Dict:
        """
        Get information about the collection.

        Returns:
            Dictionary containing collection information
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            # Check if collection is empty
            if collection_info.points_count == 0:
                print(f"Warning: Collection '{self.collection_name}' is empty")

            return {
                "name": self.collection_name,
                "vector_size": (collection_info.config.params.vectors_config.size
                               if hasattr(collection_info.config.params.vectors_config, 'size')
                               else 'unknown'),
                "count": collection_info.points_count
            }
        except Exception as e:
            print(f"Error getting collection info: {e}")
            return {}

    def check_empty_database_fallback(self) -> bool:
        """
        Check if the vector database is empty and handle fallback.

        Returns:
            True if database has content, False if empty (fallback scenario)
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            is_empty = collection_info.points_count == 0

            if is_empty:
                print(f"Warning: Vector database collection '{self.collection_name}' is empty. Using fallback response.")

            return not is_empty
        except Exception as e:
            print(f"Error checking database status: {e}")
            # Assume it's empty if we can't check
            return False

    def health_check(self) -> bool:
        """
        Check if the Qdrant client is healthy and connected.

        Returns:
            True if healthy, False otherwise
        """
        try:
            self.client.get_collections()
            return True
        except Exception:
            return False