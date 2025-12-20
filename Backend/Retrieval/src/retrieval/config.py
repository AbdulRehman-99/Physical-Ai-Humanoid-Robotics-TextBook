import os
from typing import Dict, Any

class RetrievalConfig:
    """
    Configuration class for retrieval operations.
    """

    def __init__(self):
        self.top_k = int(os.getenv("TOP_K", "10"))
        self.similarity_threshold = float(os.getenv("SIMILARITY_THRESHOLD", "0.5"))
        self.max_latency_ms = float(os.getenv("MAX_LATENCY_MS", "500"))
        self.qdrant_host = os.getenv("QDRANT_HOST", "localhost")
        self.qdrant_port = int(os.getenv("QDRANT_PORT", "6333"))
        self.qdrant_collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_content")
        # Weighting for selected-text retrieval: 70% semantic similarity, 30% exact match
        self.semantic_weight = float(os.getenv("SEMANTIC_WEIGHT", "0.7"))
        self.exact_match_weight = float(os.getenv("EXACT_MATCH_WEIGHT", "0.3"))

    def get_config(self) -> Dict[str, Any]:
        """
        Get all configuration values as a dictionary.

        Returns:
            Dictionary containing all configuration values
        """
        return {
            "top_k": self.top_k,
            "similarity_threshold": self.similarity_threshold,
            "max_latency_ms": self.max_latency_ms,
            "qdrant_host": self.qdrant_host,
            "qdrant_port": self.qdrant_port,
            "qdrant_collection_name": self.qdrant_collection_name
        }

    def update_config(self, **kwargs) -> None:
        """
        Update configuration values.

        Args:
            **kwargs: Configuration values to update
        """
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)