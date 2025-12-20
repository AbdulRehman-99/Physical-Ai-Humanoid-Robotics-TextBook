"""Unit tests for the vectorizer module."""

import unittest
from unittest.mock import Mock, patch
import numpy as np
from src.ingestion.vectorizer import Vectorizer
from src.ingestion.models import DocumentChunk, VectorEmbedding


class TestVectorizer(unittest.TestCase):
    """Test cases for Vectorizer class."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.vectorizer = Vectorizer()

    @patch('src.ingestion.embedder.Embedder.embed_texts')
    def test_generate_embeddings(self, mock_embed_texts):
        """Test generating embeddings for document chunks."""
        # Mock the embed_texts method to return known embeddings
        mock_embeddings = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]
        mock_embed_texts.return_value = mock_embeddings

        # Create test chunks
        chunks = [
            DocumentChunk(
                id="chunk_1",
                text_content="This is the first chunk of text.",
                module="module1",
                chapter="chapter1",
                section="section1",
                book_version="v1.0",
                source_file_path="test.md",
                chunk_index=0
            ),
            DocumentChunk(
                id="chunk_2",
                text_content="This is the second chunk of text.",
                module="module1",
                chapter="chapter1",
                section="section1",
                book_version="v1.0",
                source_file_path="test.md",
                chunk_index=1
            )
        ]

        # Generate embeddings
        result = self.vectorizer.generate_embeddings(chunks)

        # Verify the results
        self.assertEqual(len(result), 2)
        self.assertIsInstance(result[0], VectorEmbedding)
        self.assertEqual(result[0].chunk_id, "chunk_1")
        self.assertEqual(result[0].embedding, [0.1, 0.2, 0.3])
        self.assertEqual(result[1].chunk_id, "chunk_2")
        self.assertEqual(result[1].embedding, [0.4, 0.5, 0.6])

        # Verify that embed_texts was called with the correct text
        mock_embed_texts.assert_called_once_with([
            "This is the first chunk of text.",
            "This is the second chunk of text."
        ])

    @patch('src.ingestion.vector_store.VectorStore.upsert_vectors')
    def test_store_embeddings(self, mock_upsert_vectors):
        """Test storing embeddings in Qdrant."""
        # Mock the upsert_vectors method
        mock_upsert_vectors.return_value = True

        # Create test embeddings
        embeddings = [
            VectorEmbedding(
                chunk_id="chunk_1",
                embedding=[0.1, 0.2, 0.3],
                text_content="This is the first chunk of text.",
                metadata={"module": "module1", "chapter": "chapter1", "section": "section1"}
            ),
            VectorEmbedding(
                chunk_id="chunk_2",
                embedding=[0.4, 0.5, 0.6],
                text_content="This is the second chunk of text.",
                metadata={"module": "module1", "chapter": "chapter1", "section": "section1"}
            )
        ]

        # Store embeddings
        result = self.vectorizer.store_embeddings(embeddings)

        # Verify the result
        self.assertTrue(result)

        # Verify that upsert_vectors was called with the correct points
        expected_points = [
            {
                'id': 'chunk_1',
                'vector': [0.1, 0.2, 0.3],
                'payload': {"module": "module1", "chapter": "chapter1", "section": "section1"}
            },
            {
                'id': 'chunk_2',
                'vector': [0.4, 0.5, 0.6],
                'payload': {"module": "module1", "chapter": "chapter1", "section": "section1"}
            }
        ]
        mock_upsert_vectors.assert_called_once_with(expected_points)

    def test_validate_embedding_quality_valid(self):
        """Test validating a valid embedding."""
        embedding = VectorEmbedding(
            chunk_id="chunk_1",
            embedding=[0.1, 0.2, 0.3, 0.4],
            text_content="Test content",
            metadata={}
        )

        result = self.vectorizer.validate_embedding_quality(embedding)
        self.assertTrue(result)

    def test_validate_embedding_quality_invalid_empty(self):
        """Test validating an embedding with no vector data."""
        embedding = VectorEmbedding(
            chunk_id="chunk_1",
            embedding=[],
            text_content="Test content",
            metadata={}
        )

        result = self.vectorizer.validate_embedding_quality(embedding)
        self.assertFalse(result)

    def test_validate_embedding_quality_invalid_wrong_dimension(self):
        """Test validating an embedding with wrong dimension."""
        # Temporarily change the setting for this test
        from src.config.settings import settings
        original_dim = settings.cohere_embedding_dimensions
        settings.cohere_embedding_dimensions = 10  # Set to a different dimension

        embedding = VectorEmbedding(
            chunk_id="chunk_1",
            embedding=[0.1, 0.2, 0.3, 0.4],  # Only 4 dimensions
            text_content="Test content",
            metadata={}
        )

        result = self.vectorizer.validate_embedding_quality(embedding)
        self.assertFalse(result)

        # Restore original dimension
        settings.cohere_embedding_dimensions = original_dim

    def test_validate_embedding_quality_invalid_nan(self):
        """Test validating an embedding with NaN values."""
        embedding = VectorEmbedding(
            chunk_id="chunk_1",
            embedding=[0.1, float('nan'), 0.3, 0.4],
            text_content="Test content",
            metadata={}
        )

        result = self.vectorizer.validate_embedding_quality(embedding)
        self.assertFalse(result)

    def test_validate_embedding_quality_invalid_infinity(self):
        """Test validating an embedding with infinity values."""
        embedding = VectorEmbedding(
            chunk_id="chunk_1",
            embedding=[0.1, float('inf'), 0.3, 0.4],
            text_content="Test content",
            metadata={}
        )

        result = self.vectorizer.validate_embedding_quality(embedding)
        self.assertFalse(result)

    def test_validate_embeddings_quality_all_valid(self):
        """Test validating a list of valid embeddings."""
        embeddings = [
            VectorEmbedding(
                chunk_id="chunk_1",
                embedding=[0.1, 0.2, 0.3, 0.4],
                text_content="Test content 1",
                metadata={}
            ),
            VectorEmbedding(
                chunk_id="chunk_2",
                embedding=[0.5, 0.6, 0.7, 0.8],
                text_content="Test content 2",
                metadata={}
            )
        ]

        result = self.vectorizer.validate_embeddings_quality(embeddings)
        self.assertTrue(result['quality_ok'])
        self.assertEqual(result['valid_embeddings'], 2)
        self.assertEqual(result['invalid_embeddings'], 0)

    def test_validate_embeddings_quality_some_invalid(self):
        """Test validating a list with some invalid embeddings."""
        embeddings = [
            VectorEmbedding(
                chunk_id="chunk_1",
                embedding=[0.1, 0.2, 0.3, 0.4],
                text_content="Test content 1",
                metadata={}
            ),
            VectorEmbedding(
                chunk_id="chunk_2",
                embedding=[],  # Invalid - empty
                text_content="Test content 2",
                metadata={}
            )
        ]

        result = self.vectorizer.validate_embeddings_quality(embeddings)
        self.assertFalse(result['quality_ok'])
        self.assertEqual(result['valid_embeddings'], 1)
        self.assertEqual(result['invalid_embeddings'], 1)
        self.assertEqual(result['invalid_embedding_ids'], ['chunk_2'])


if __name__ == '__main__':
    unittest.main()