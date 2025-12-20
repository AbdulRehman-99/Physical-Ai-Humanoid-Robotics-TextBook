"""Integration tests for the vectorizer module."""

import unittest
from unittest.mock import patch, Mock
from src.ingestion.vectorizer import Vectorizer, create_embedding_pipeline
from src.ingestion.models import DocumentChunk, VectorEmbedding


class TestVectorizerIntegration(unittest.TestCase):
    """Integration tests for Vectorizer class."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.vectorizer = Vectorizer()

    @patch('src.ingestion.embedder.Embedder.embed_texts')
    @patch('src.ingestion.vector_store.VectorStore.upsert_vectors')
    def test_process_and_store_chunks_success(self, mock_upsert_vectors, mock_embed_texts):
        """Test successful processing and storage of document chunks."""
        # Mock the embed_texts method to return known embeddings
        mock_embeddings = [[0.1, 0.2, 0.3, 0.4], [0.5, 0.6, 0.7, 0.8]]
        mock_embed_texts.return_value = mock_embeddings

        # Mock the upsert_vectors method to return success
        mock_upsert_vectors.return_value = True

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

        # Process and store chunks
        result = self.vectorizer.process_and_store_chunks(chunks)

        # Verify the results
        self.assertTrue(result)

        # Verify that embed_texts was called with the correct text
        mock_embed_texts.assert_called_once_with([
            "This is the first chunk of text.",
            "This is the second chunk of text."
        ])

        # Verify that upsert_vectors was called with the correct points
        expected_points = [
            {
                'id': 'chunk_1',
                'vector': [0.1, 0.2, 0.3, 0.4],
                'payload': {
                    'module': 'module1',
                    'chapter': 'chapter1',
                    'section': 'section1',
                    'book_version': 'v1.0',
                    'source_file_path': 'test.md',
                    'chunk_index': 0
                }
            },
            {
                'id': 'chunk_2',
                'vector': [0.5, 0.6, 0.7, 0.8],
                'payload': {
                    'module': 'module1',
                    'chapter': 'chapter1',
                    'section': 'section1',
                    'book_version': 'v1.0',
                    'source_file_path': 'test.md',
                    'chunk_index': 1
                }
            }
        ]
        mock_upsert_vectors.assert_called_once_with(expected_points)

    @patch('src.ingestion.embedder.Embedder.embed_texts')
    @patch('src.ingestion.vector_store.VectorStore.upsert_vectors')
    def test_process_and_store_chunks_failure(self, mock_upsert_vectors, mock_embed_texts):
        """Test handling of failure when storing embeddings."""
        # Mock the embed_texts method to return known embeddings
        mock_embeddings = [[0.1, 0.2, 0.3, 0.4]]
        mock_embed_texts.return_value = mock_embeddings

        # Mock the upsert_vectors method to return failure
        mock_upsert_vectors.return_value = False

        # Create test chunk
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
            )
        ]

        # Process and store chunks
        result = self.vectorizer.process_and_store_chunks(chunks)

        # Verify the results
        self.assertFalse(result)

    @patch('src.ingestion.embedder.Embedder.embed_texts')
    @patch('src.ingestion.vector_store.VectorStore.upsert_vectors')
    def test_embedding_pipeline_integration(self, mock_upsert_vectors, mock_embed_texts):
        """Test the complete embedding pipeline."""
        # Mock the embed_texts method to return known embeddings
        mock_embeddings = [[0.1, 0.2, 0.3, 0.4], [0.5, 0.6, 0.7, 0.8]]
        mock_embed_texts.return_value = mock_embeddings

        # Mock the upsert_vectors method to return success
        mock_upsert_vectors.return_value = True

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

        # Create and run the embedding pipeline
        pipeline = create_embedding_pipeline()
        results = pipeline(chunks, validate_quality=True)

        # Verify the results
        self.assertEqual(results['total_chunks_processed'], 2)
        self.assertEqual(results['total_embeddings_generated'], 2)
        self.assertTrue(results['storage_success'])
        self.assertIsNotNone(results['quality_report'])

    @patch('src.ingestion.embedder.Embedder.embed_texts')
    def test_embedding_quality_validation(self, mock_embed_texts):
        """Test embedding quality validation."""
        # Mock the embed_texts method to return an embedding with wrong dimension
        mock_embeddings = [[0.1, 0.2]]  # Only 2 dimensions instead of expected 4
        mock_embed_texts.return_value = mock_embeddings

        # Create test chunk
        chunks = [
            DocumentChunk(
                id="chunk_1",
                text_content="This is a test chunk.",
                module="module1",
                chapter="chapter1",
                section="section1",
                book_version="v1.0",
                source_file_path="test.md",
                chunk_index=0
            )
        ]

        # Temporarily change the expected dimension for this test
        from src.config.settings import settings
        original_dim = settings.cohere_embedding_dimensions
        settings.cohere_embedding_dimensions = 4  # Set to expected 4 dimensions

        # Process embeddings to trigger quality validation
        embeddings = self.vectorizer.generate_embeddings(chunks)
        quality_report = self.vectorizer.validate_embeddings_quality(embeddings)

        # Verify the quality report shows the issue
        self.assertFalse(quality_report['quality_ok'])
        self.assertEqual(quality_report['invalid_embeddings'], 1)

        # Restore original dimension
        settings.cohere_embedding_dimensions = original_dim


if __name__ == '__main__':
    unittest.main()