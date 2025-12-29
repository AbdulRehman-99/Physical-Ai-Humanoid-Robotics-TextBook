import os
from typing import List, Dict, Optional
from qdrant_client import QdrantClient
from qdrant_client.http.models import models
from dotenv import load_dotenv
import sys
from pathlib import Path

# Load environment variables
load_dotenv()

# Try to load environment variables from the project root directory
# The .env file is located at the project root
# Navigate up from this file: Backend/Retrieval/src/retrieval/client.py -> root
env_path = Path(__file__).parent.parent.parent.parent.parent / ".env"

# Load environment variables from the root directory if it exists
if env_path.exists():
    load_dotenv(dotenv_path=env_path)
else:
    # Try to load from the standard location
    load_dotenv()


class QdrantRetrievalClient:
    """
    Qdrant client for retrieval operations on book content vector database.
    """

    def __init__(self):
        self.host = os.getenv("QDRANT_HOST", "localhost")
        self.port = int(os.getenv("QDRANT_PORT", "6333"))
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_embeddings")
        self.url = os.getenv("QDRANT_URL", "")
        self.api_key = os.getenv("QDRANT_API_KEY", "")

        # Initialize Qdrant client - use URL for cloud instances, host/port for local
        if self.url and self.api_key:
            # Use URL for Qdrant Cloud
            self.client = QdrantClient(
                url=self.url,
                api_key=self.api_key,
                https=True
            )
        else:
            # Use host/port for local Qdrant
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

            # Use the correct Qdrant client method for search
            # For newer versions of qdrant-client, use query_points
            search_results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=top_k,
                query_filter=query_filter
            )

            # Return the points from the search results
            return search_results.points
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