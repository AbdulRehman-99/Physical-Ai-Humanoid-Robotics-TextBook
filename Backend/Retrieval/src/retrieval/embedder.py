"""Module for generating text embeddings using Cohere API."""
import os
import time
from typing import List, Optional
import cohere
from .config import RetrievalConfig
from .logger import RetrievalLogger
import logging
import random

logger = logging.getLogger(__name__)

class Embedder:
    """Class to handle text embedding generation using Cohere API."""

    def __init__(self):
        """Initialize the embedder with Cohere client and rate limiting parameters."""
        api_key = os.getenv("COHERE_API_KEY")
        if not api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")

        try:
            self.client = cohere.Client(
                api_key=api_key
            )
        except Exception as e:
            logger.error(f"Failed to initialize Cohere client: {e}")
            raise

        self.config = RetrievalConfig()
        self.max_retries = 3
        self.base_delay = 1.0

    def embed_texts(self, texts: List[str], model: str = "embed-multilingual-v3.0",
                   input_type: str = "search_document") -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere API with rate limiting.

        Args:
            texts: List of text strings to embed
            model: Cohere model to use for embeddings (default: multilingual-v3.0)
            input_type: Type of input for the model (default: search_document)

        Returns:
            List of embedding vectors (each vector is a list of floats)
        """
        if not texts:
            return []

        # Handle large batches by splitting them into smaller chunks
        # Cohere API has limits on batch size, so we'll process in chunks of 96
        chunk_size = 96  # Safe size under Cohere's limits
        all_embeddings = []

        for i in range(0, len(texts), chunk_size):
            text_chunk = texts[i:i + chunk_size]
            chunk_embeddings = self._embed_chunk_with_retry(
                text_chunk, model, input_type
            )
            all_embeddings.extend(chunk_embeddings)

        return all_embeddings

    def _embed_chunk_with_retry(self, texts: List[str], model: str,
                               input_type: str) -> List[List[float]]:
        """
        Embed a single chunk of texts with retry logic.

        Args:
            texts: List of text strings to embed (should be within API limits)
            model: Cohere model to use for embeddings
            input_type: Type of input for the model

        Returns:
            List of embedding vectors
        """
        last_exception = None

        for attempt in range(self.max_retries):
            try:
                response = self.client.embed(
                    texts=texts,
                    model=model,
                    input_type=input_type
                )

                # Extract embeddings from response
                embeddings = [embedding for embedding in response.embeddings]
                logger.info(f"Successfully generated {len(embeddings)} embeddings in attempt {attempt + 1}")
                return embeddings

            except Exception as e:
                last_exception = e
                error_msg = str(e).lower()

                # Check if this is a rate limit error to log it specifically
                if any(rate_limit_indicator in error_msg for rate_limit_indicator in
                       ['rate limit', 'quota', 'too many requests', '429', 'billing', 'credit', 'exceeded']):
                    logger.warning(f"Quota/rate limit error in attempt {attempt + 1}: {e}")
                    # If it's a quota issue, additional retries won't help, so break early
                    if any(quota_indicator in error_msg for quota_indicator in ['quota', 'billing', 'credit', 'exceeded']):
                        logger.error(f"Quota exceeded - no point in retrying: {e}")
                        break
                else:
                    logger.warning(f"Cohere API error in attempt {attempt + 1}: {e}")

                if attempt < self.max_retries - 1:  # don't sleep on the last attempt
                    # Exponential backoff with jitter: delay = base_delay * (2 ^ attempt)
                    delay = self.base_delay * (2 ** attempt) + random.uniform(0, 1)
                    logger.info(f"Retrying in {delay:.2f} seconds...")
                    time.sleep(delay)
                else:
                    logger.error(f"All {self.max_retries} attempts failed. Last error: {e}")

        # If all retries failed due to quota issues, return empty embeddings as fallback
        logger.warning("Embedding generation failed due to API issues, returning empty embeddings")
        return [[] for _ in range(len(texts))]

    def embed_single_text(self, text: str, model: str = "embed-multilingual-v3.0",
                         input_type: str = "search_document") -> List[float]:
        """
        Generate embedding for a single text using Cohere API.

        Args:
            text: Single text string to embed
            model: Cohere model to use for embeddings (default: multilingual-v3.0)
            input_type: Type of input for the model (default: search_document)

        Returns:
            Single embedding vector (list of floats)
        """
        embeddings = self.embed_texts([text], model, input_type)
        return embeddings[0] if embeddings else []

    def validate_embedding_dimensions(self, embedding: List[float],
                                    expected_dim: Optional[int] = 1024) -> bool:
        """
        Validate that an embedding has the expected dimensions.

        Args:
            embedding: Embedding vector to validate
            expected_dim: Expected dimension (default 1024 for v3 model)

        Returns:
            True if embedding has valid dimensions, False otherwise
        """
        if not embedding:
            # Empty embeddings are considered invalid, but we log this separately
            # as it may be due to API failures rather than invalid input
            logger.info(f"Embedding is empty - likely due to API failure")
            return False

        if len(embedding) != expected_dim:
            logger.warning(f"Embedding dimension mismatch: expected {expected_dim}, got {len(embedding)}")
            return False

        # Check for NaN or infinity values
        for value in embedding:
            if not (float('-inf') < value < float('inf')):
                logger.warning(f"Embedding contains invalid values (NaN or infinity)")
                return False

        return True