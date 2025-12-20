"""Module for handling vector storage operations with Qdrant."""

from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from src.config.settings import settings
from src.ingestion.exceptions import VectorStorageError
import logging

logger = logging.getLogger(__name__)

class VectorStore:
    """Class to handle vector storage operations with Qdrant."""

    def __init__(self):
        """Initialize the Qdrant client and set up connection."""
        try:
            self.client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
                # Set timeout and other connection parameters
                timeout=30
            )
            self.collection_name = settings.qdrant_collection_name
        except Exception as e:
            logger.error(f"Failed to initialize Qdrant client: {e}")
            raise VectorStorageError(f"Failed to connect to Qdrant: {e}")

    def ensure_collection_exists(self, vector_size: int = 1024) -> bool:
        """
        Ensure that the collection exists in Qdrant.

        Args:
            vector_size: Size of the vectors to be stored (default 1024 for Cohere embeddings)

        Returns:
            True if collection exists or was created successfully
        """
        try:
            # Check if collection already exists
            collections = self.client.get_collections()
            collection_names = [collection.name for collection in collections.collections]

            if self.collection_name not in collection_names:
                # Create the collection
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=vector_size,
                        distance=Distance.COSINE
                    )
                )
                logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"Qdrant collection already exists: {self.collection_name}")

            return True
        except Exception as e:
            logger.error(f"Failed to ensure collection exists: {e}")
            raise VectorStorageError(f"Failed to ensure collection exists: {e}")

    def upsert_vectors(self, points: List[Dict[str, Any]]) -> bool:
        """
        Upsert vectors into the Qdrant collection.

        Args:
            points: List of dictionaries containing vector data and metadata
                   Each dict should have: id, vector, payload (metadata)

        Returns:
            True if upsert was successful
        """
        try:
            # Ensure the collection exists before upserting
            # Using the expected vector size for Cohere embeddings (1024 dimensions)
            self.ensure_collection_exists(vector_size=1024)

            # Prepare the points for upsert
            qdrant_points = []
            for point in points:
                qdrant_point = models.PointStruct(
                    id=point["id"],
                    vector=point["vector"],
                    payload=point["payload"]
                )
                qdrant_points.append(qdrant_point)

            # Upsert the points
            self.client.upsert(
                collection_name=self.collection_name,
                points=qdrant_points
            )

            logger.info(f"Successfully upserted {len(points)} vectors to collection {self.collection_name}")
            return True
        except Exception as e:
            logger.error(f"Failed to upsert vectors: {e}")
            raise VectorStorageError(f"Failed to upsert vectors: {e}")

    def search_vectors(self, query_vector: List[float], top_k: int = 10, min_score: float = 0.0, filters: Optional[Dict] = None) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in the Qdrant collection.

        Args:
            query_vector: Vector to search for similarity
            top_k: Number of top results to return
            min_score: Minimum relevance score threshold
            filters: Optional filter for metadata fields

        Returns:
            List of dictionaries containing search results with metadata
        """
        try:
            # Prepare filters if provided
            search_filter = None
            if filters:
                filter_conditions = []
                for key, value in filters.items():
                    filter_conditions.append(
                        models.FieldCondition(
                            key=key,
                            match=models.MatchValue(value=value)
                        )
                    )

                if filter_conditions:
                    search_filter = models.Filter(
                        must=filter_conditions
                    )

            # Perform the search
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                query_filter=search_filter,
                limit=top_k,
                score_threshold=min_score  # Add minimum score threshold
            )

            # Format results
            results = []
            for hit in search_results:
                result = {
                    "id": hit.id,
                    "score": hit.score,
                    "payload": hit.payload,
                    "vector": hit.vector if hasattr(hit, 'vector') else None
                }
                results.append(result)

            logger.debug(f"Search returned {len(results)} results")
            return results
        except Exception as e:
            logger.error(f"Failed to search vectors: {e}")
            raise VectorStorageError(f"Failed to search vectors: {e}")

    def get_vector_count(self) -> int:
        """
        Get the total count of vectors in the collection.

        Returns:
            Number of vectors in the collection
        """
        try:
            count = self.client.count(
                collection_name=self.collection_name
            )
            return count.count
        except Exception as e:
            logger.error(f"Failed to get vector count: {e}")
            raise VectorStorageError(f"Failed to get vector count: {e}")