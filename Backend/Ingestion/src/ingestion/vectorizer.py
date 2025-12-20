"""Module for generating embeddings using Cohere and storing in Qdrant."""

import time
import asyncio
from typing import List, Dict, Any, Optional
from src.ingestion.models import DocumentChunk, VectorEmbedding
from src.ingestion.embedder import Embedder
from src.ingestion.vector_store import VectorStore
from src.config.settings import settings
import logging

logger = logging.getLogger(__name__)


class Vectorizer:
    """Class to handle the generation and storage of vector embeddings."""

    def __init__(self):
        """Initialize the vectorizer with Cohere embedder and Qdrant vector store."""
        self.embedder = Embedder()
        self.vector_store = VectorStore()

    def generate_embeddings(self, chunks: List[DocumentChunk]) -> List[VectorEmbedding]:
        """
        Generate embeddings for a list of document chunks using Cohere.

        Args:
            chunks: List of DocumentChunk objects to generate embeddings for

        Returns:
            List of VectorEmbedding objects with generated embeddings
        """
        logger.info(f"Generating embeddings for {len(chunks)} chunks")

        # Extract text content from chunks for embedding
        texts = [chunk.text_content for chunk in chunks]

        # Generate embeddings using Cohere
        embeddings = self.embedder.embed_texts(texts)

        # Create VectorEmbedding objects with the generated embeddings
        vector_embeddings = []
        for i, chunk in enumerate(chunks):
            vector_embedding = VectorEmbedding(
                chunk_id=chunk.id,
                embedding=embeddings[i],
                text_content=chunk.text_content,
                metadata={
                    'module': chunk.module,
                    'chapter': chunk.chapter,
                    'section': chunk.section,
                    'book_version': chunk.book_version,
                    'source_file_path': chunk.source_file_path,
                    'chunk_index': chunk.chunk_index,
                    **chunk.metadata  # Include any additional metadata from the chunk
                }
            )
            vector_embeddings.append(vector_embedding)

        logger.info(f"Successfully generated embeddings for {len(chunks)} chunks")
        return vector_embeddings

    def store_embeddings(self, embeddings: List[VectorEmbedding]) -> bool:
        """
        Store vector embeddings in Qdrant with their metadata.

        Args:
            embeddings: List of VectorEmbedding objects to store

        Returns:
            True if storage was successful, False otherwise
        """
        logger.info(f"Storing {len(embeddings)} embeddings in Qdrant")

        try:
            # Convert VectorEmbedding objects to the format expected by VectorStore
            points = []
            for embedding in embeddings:
                # Generate a proper ID for Qdrant (using UUID format)
                import uuid
                point_id = str(uuid.uuid4())

                point = {
                    'id': point_id,
                    'vector': embedding.embedding,
                    'payload': {
                        **embedding.metadata,
                        'original_chunk_id': embedding.chunk_id  # Keep original ID in metadata
                    }
                }
                points.append(point)

            # Store embeddings in Qdrant
            success = self.vector_store.upsert_vectors(points)

            if success:
                logger.info(f"Successfully stored {len(embeddings)} embeddings in Qdrant")
            else:
                logger.error("Failed to store embeddings in Qdrant")

            return success

        except Exception as e:
            logger.error(f"Error storing embeddings in Qdrant: {e}")
            return False

    def process_and_store_chunks(self, chunks: List[DocumentChunk]) -> bool:
        """
        Process document chunks by generating embeddings and storing them in Qdrant.

        Args:
            chunks: List of DocumentChunk objects to process

        Returns:
            True if processing and storage were successful, False otherwise
        """
        try:
            # Generate embeddings for the chunks
            embeddings = self.generate_embeddings(chunks)

            # Store the embeddings in Qdrant
            success = self.store_embeddings(embeddings)

            if success:
                logger.info(f"Successfully processed and stored embeddings for {len(chunks)} chunks")
            else:
                logger.error(f"Failed to process and store embeddings for {len(chunks)} chunks")

            return success

        except Exception as e:
            logger.error(f"Error in process_and_store_chunks: {e}")
            return False

    def validate_embedding_quality(self, embedding: VectorEmbedding) -> bool:
        """
        Validate the quality of a generated embedding.

        Args:
            embedding: VectorEmbedding object to validate

        Returns:
            True if embedding quality is acceptable, False otherwise
        """
        # Check if embedding vector is not empty
        if not embedding.embedding:
            logger.warning(f"Embedding for chunk {embedding.chunk_id} has no vector data")
            return False

        # Check if embedding vector has the expected dimension
        expected_dim = settings.cohere_embedding_dimensions
        if len(embedding.embedding) != expected_dim:
            logger.warning(f"Embedding for chunk {embedding.chunk_id} has unexpected dimension: {len(embedding.embedding)}, expected: {expected_dim}")
            return False

        # Check if embedding contains valid numerical values (no NaN or infinity)
        import math
        for value in embedding.embedding:
            if math.isnan(value) or math.isinf(value):
                logger.warning(f"Embedding for chunk {embedding.chunk_id} contains invalid values (NaN or infinity)")
                return False

        return True

    def validate_embeddings_quality(self, embeddings: List[VectorEmbedding]) -> Dict[str, Any]:
        """
        Validate the quality of a list of embeddings.

        Args:
            embeddings: List of VectorEmbedding objects to validate

        Returns:
            Dictionary with validation results
        """
        total_embeddings = len(embeddings)
        valid_embeddings = 0
        invalid_embedding_ids = []

        for embedding in embeddings:
            if self.validate_embedding_quality(embedding):
                valid_embeddings += 1
            else:
                invalid_embedding_ids.append(embedding.chunk_id)

        quality_ok = valid_embeddings == total_embeddings
        quality_report = {
            'total_embeddings': total_embeddings,
            'valid_embeddings': valid_embeddings,
            'invalid_embeddings': total_embeddings - valid_embeddings,
            'quality_ok': quality_ok,
            'invalid_embedding_ids': invalid_embedding_ids
        }

        if not quality_ok:
            logger.warning(f"Embedding quality issues found: {quality_report}")
        else:
            logger.info(f"All embeddings passed quality validation: {quality_report}")

        return quality_report


def create_embedding_pipeline():
    """
    Create an embedding pipeline function that orchestrates the embedding generation and storage.

    Returns:
        A function that can be called to run the embedding pipeline
    """
    def run_embedding_pipeline(chunks: List[DocumentChunk], validate_quality: bool = True):
        """
        Run the complete embedding pipeline.

        Args:
            chunks: List of DocumentChunk objects to process
            validate_quality: Whether to validate embedding quality before storing

        Returns:
            Dictionary with pipeline results
        """
        logger.info("Starting embedding pipeline...")

        # Initialize components
        vectorizer = Vectorizer()

        # Generate embeddings
        embeddings = vectorizer.generate_embeddings(chunks)

        # Validate quality if requested
        if validate_quality:
            quality_report = vectorizer.validate_embeddings_quality(embeddings)
            if not quality_report['quality_ok']:
                logger.warning(f"Quality issues detected: {quality_report['invalid_embedding_ids']}")

        # Store embeddings in Qdrant
        success = vectorizer.store_embeddings(embeddings)

        # Prepare results
        results = {
            'total_chunks_processed': len(chunks),
            'total_embeddings_generated': len(embeddings),
            'storage_success': success,
            'quality_report': quality_report if validate_quality else None
        }

        if success:
            logger.info(f"Embedding pipeline completed successfully: {results}")
        else:
            logger.error(f"Embedding pipeline failed: {results}")

        return results

    return run_embedding_pipeline