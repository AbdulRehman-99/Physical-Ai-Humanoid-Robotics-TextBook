import os
from typing import Optional
from dataclasses import dataclass
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

@dataclass
class Settings:
    """Application settings loaded from environment variables."""

    # API Keys
    cohere_api_key: str = os.getenv("COHERE_API_KEY", "")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    qdrant_url: str = os.getenv("QDRANT_URL", "")

    # Application settings
    log_level: str = os.getenv("LOG_LEVEL", "INFO")

    # Processing settings
    chunk_size: int = int(os.getenv("CHUNK_SIZE", "512"))
    chunk_overlap: int = int(os.getenv("CHUNK_OVERLAP", "100"))
    max_document_size: int = int(os.getenv("MAX_DOCUMENT_SIZE", "10485760"))  # 10MB in bytes

    # API settings
    cohere_retry_attempts: int = int(os.getenv("COHERE_RETRY_ATTEMPTS", "5"))
    cohere_base_delay: float = float(os.getenv("COHERE_BASE_DELAY", "1.0"))
    cohere_embedding_dimensions: int = int(os.getenv("COHERE_EMBEDDING_DIMENSIONS", "1024"))  # Default for multilingual v3

    # Qdrant settings
    qdrant_collection_name: str = os.getenv("QDRANT_COLLECTION_NAME", "book_embeddings")

    def __post_init__(self):
        """Validate required settings after initialization."""
        if not self.cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")
        if not self.qdrant_api_key:
            raise ValueError("QDRANT_API_KEY environment variable is required")
        if not self.qdrant_url:
            raise ValueError("QDRANT_URL environment variable is required")

# Global settings instance
settings = Settings()