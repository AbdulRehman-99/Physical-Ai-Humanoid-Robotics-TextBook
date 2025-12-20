"""Module for generating text embeddings using Cohere API with rate limiting."""

import time
import asyncio
from typing import List, Optional
import cohere
from src.config.settings import settings
from src.ingestion.exceptions import APIClientError
import logging
import random

logger = logging.getLogger(__name__)

class Embedder:
    """Class to handle text embedding generation using Cohere API."""

    def __init__(self):
        """Initialize the embedder with Cohere client and rate limiting parameters."""
        try:
            self.client = cohere.Client(
                api_key=settings.cohere_api_key
            )
        except Exception as e:
            logger.error(f"Failed to initialize Cohere client: {e}")
            raise APIClientError(f"Failed to initialize Cohere client: {e}", api_name="Cohere")

        self.max_retries = settings.cohere_retry_attempts
        self.base_delay = settings.cohere_base_delay

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
                logger.warning(f"Cohere API error in attempt {attempt + 1}: {e}")

                if attempt < self.max_retries - 1:  # Don't sleep on the last attempt
                    # Exponential backoff with jitter: delay = base_delay * (2 ^ attempt)
                    delay = self.base_delay * (2 ** attempt) + random.uniform(0, 1)
                    logger.info(f"Retrying in {delay:.2f} seconds...")
                    time.sleep(delay)
                else:
                    logger.error(f"All {self.max_retries} attempts failed. Last error: {e}")
                    raise APIClientError(f"Failed to generate embeddings after {self.max_retries} attempts: {e}",
                                       api_name="Cohere")

        # If all retries failed, raise the last exception
        if last_exception:
            raise last_exception
        else:
            raise Exception("Embedding failed after all retry attempts")

    def embed_single_text(self, text: str, model: str = "embed-multilingual-v3.0",
                         input_type: str = "search_document") -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Single text string to embed
            model: Cohere model to use for embeddings
            input_type: Type of input for the model

        Returns:
            Single embedding vector (list of floats)
        """
        embeddings = self.embed_texts([text], model, input_type)
        return embeddings[0] if embeddings else []

    def validate_embedding_dimensions(self, embedding: List[float],
                                    expected_dim: Optional[int] = None) -> bool:
        """
        Validate that an embedding has the expected dimensions.

        Args:
            embedding: Embedding vector to validate
            expected_dim: Expected dimension (uses setting if None)

        Returns:
            True if embedding has valid dimensions, False otherwise
        """
        if expected_dim is None:
            expected_dim = settings.cohere_embedding_dimensions

        if not embedding:
            logger.warning(f"Embedding is empty")
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

    def validate_embeddings_batch(self, embeddings: List[List[float]],
                                expected_dim: Optional[int] = None) -> bool:
        """
        Validate that a batch of embeddings have the expected dimensions.

        Args:
            embeddings: List of embedding vectors to validate
            expected_dim: Expected dimension (uses setting if None)

        Returns:
            True if all embeddings have valid dimensions, False otherwise
        """
        for i, embedding in enumerate(embeddings):
            if not self.validate_embedding_dimensions(embedding, expected_dim):
                logger.warning(f"Invalid embedding at index {i}")
                return False

        return True