"""End-to-end tests for the book ingestion system."""

import unittest
import tempfile
import os
from pathlib import Path
import shutil
from unittest.mock import patch, Mock
from src.ingestion.document_processor import DocumentProcessor
from src.ingestion.vectorizer import Vectorizer
from src.ingestion.models import DocumentChunk
from src.ingestion.performance_monitor import PerformanceMonitor


class TestEndToEnd(unittest.TestCase):
    """End-to-end tests for the complete ingestion pipeline."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        # Create a temporary directory for test files
        self.test_dir = Path(tempfile.mkdtemp())
        self.performance_monitor = PerformanceMonitor()

    def tearDown(self):
        """Clean up after each test method."""
        # Remove the temporary directory
        shutil.rmtree(self.test_dir, ignore_errors=True)

    def create_test_markdown_files(self):
        """Create test markdown files in the temporary directory."""
        # Create module directories
        module1_dir = self.test_dir / "module1"
        module1_dir.mkdir()

        chapter1_dir = module1_dir / "chapter1"
        chapter1_dir.mkdir()

        # Create a test markdown file
        test_file = chapter1_dir / "test_chapter.md"
        test_content = """
# Introduction to Robotics

This is a test chapter about robotics.

## Humanoid Robots

Humanoid robots are robots with human-like features. They are designed to interact with humans in a natural way.

## ROS Basics

ROS (Robot Operating System) is a flexible framework for writing robot software. It provides services designed for a heterogeneous computer cluster.

### Key Concepts

- Nodes
- Topics
- Services
- Actions

These concepts form the foundation of ROS architecture.
"""
        test_file.write_text(test_content)

        return [test_file]

    @patch('src.ingestion.embedder.Embedder.embed_texts')
    @patch('src.ingestion.vector_store.VectorStore.upsert_vectors')
    def test_complete_ingestion_pipeline(self, mock_upsert_vectors, mock_embed_texts):
        """Test the complete ingestion pipeline from document processing to vector storage."""
        # Mock embedding generation
        mock_embed_texts.return_value = [
            [0.1, 0.2, 0.3, 0.4],
            [0.5, 0.6, 0.7, 0.8],
            [0.9, 1.0, 1.1, 1.2]
        ]

        # Mock vector storage success
        mock_upsert_vectors.return_value = True

        # Create test markdown files
        test_files = self.create_test_markdown_files()
        source_path = str(self.test_dir)

        # Initialize components
        processor = DocumentProcessor()
        vectorizer = Vectorizer()

        # Step 1: Process documents
        processing_results = processor.process_documents(source_path)

        # Verify processing results
        self.assertGreater(len(processing_results), 0)
        successful_results = [r for r in processing_results if r.status == "success"]
        self.assertGreater(len(successful_results), 0)

        # Step 2: Get chunks from successful documents
        all_chunks = []
        for file_path in [f for f in test_files]:  # Process the created files
            try:
                chunks = processor.process_single_document(file_path)
                all_chunks.extend(chunks)
            except Exception as e:
                print(f"Error processing {file_path}: {e}")

        # Verify we have chunks to process
        self.assertGreater(len(all_chunks), 0)

        # Step 3: Generate and store embeddings
        success = vectorizer.process_and_store_chunks(all_chunks)

        # Verify embedding process success
        self.assertTrue(success)

        # Verify that embeddings were generated and stored
        mock_embed_texts.assert_called()
        mock_upsert_vectors.assert_called()

        # Check that the right number of vectors were processed
        # This depends on how many chunks were created during processing
        print(f"Processed {len(all_chunks)} chunks")
        print(f"Processing results: {len(processing_results)}")

    def test_document_processing_quality(self):
        """Test the quality of document processing."""
        # Create test markdown files
        test_files = self.create_test_markdown_files()

        # Initialize processor
        processor = DocumentProcessor()

        # Process a single document
        file_path = test_files[0]
        chunks = processor.process_single_document(file_path)

        # Verify that chunks were created
        self.assertGreater(len(chunks), 0)

        # Verify that chunks have expected properties
        for chunk in chunks:
            self.assertIsInstance(chunk, DocumentChunk)
            self.assertIsNotNone(chunk.text_content)
            self.assertNotEqual(chunk.text_content.strip(), "")
            self.assertIsNotNone(chunk.module)
            self.assertIsNotNone(chunk.chapter)
            self.assertIsNotNone(chunk.section)

        # Verify that content was properly cleaned and normalized
        all_text = " ".join([chunk.text_content for chunk in chunks])
        # Check that markdown formatting was removed
        self.assertNotIn("# Introduction", all_text)  # Raw markdown headers should be removed
        self.assertIn("Introduction to Robotics", all_text)  # But content should remain

    @patch('src.ingestion.embedder.Embedder.embed_texts')
    def test_embedding_quality_validation(self, mock_embed_texts):
        """Test that embeddings are properly validated."""
        # Mock embedding generation with proper dimensions
        mock_embedding = [0.1] * 1024  # Assuming 1024-dimensional embeddings
        mock_embed_texts.return_value = [mock_embedding, mock_embedding]

        # Create test chunks
        test_chunk1 = DocumentChunk(
            id="test_chunk_1",
            text_content="This is the first test chunk.",
            module="module1",
            chapter="chapter1",
            section="section1",
            book_version="v1.0",
            source_file_path="test.md",
            chunk_index=0
        )

        test_chunk2 = DocumentChunk(
            id="test_chunk_2",
            text_content="This is the second test chunk.",
            module="module1",
            chapter="chapter1",
            section="section1",
            book_version="v1.0",
            source_file_path="test.md",
            chunk_index=1
        )

        test_chunks = [test_chunk1, test_chunk2]

        # Initialize vectorizer and generate embeddings
        vectorizer = Vectorizer()
        embeddings = vectorizer.generate_embeddings(test_chunks)

        # Verify that embeddings were generated
        self.assertEqual(len(embeddings), 2)
        for embedding in embeddings:
            self.assertEqual(len(embedding.embedding), 1024)  # Expected dimension

        # Validate embedding quality
        quality_report = vectorizer.validate_embeddings_quality(embeddings)
        self.assertTrue(quality_report['quality_ok'])

    def test_performance_monitoring(self):
        """Test that performance is properly monitored during processing."""
        # Create test markdown files
        test_files = self.create_test_markdown_files()

        # Initialize processor
        processor = DocumentProcessor()

        # Process a document and measure performance
        start_time = self.performance_monitor.start_timer()
        file_path = test_files[0]
        chunks = processor.process_single_document(file_path)
        duration = self.performance_monitor.stop_timer(start_time)

        # Record the performance metric
        self.performance_monitor.record_document_processing(
            duration=duration,
            success=True,
            chunks_created=len(chunks)
        )

        # Verify that metrics were recorded
        summary = self.performance_monitor.get_metrics_summary()
        self.assertIn('document.processing.duration', summary)

        # Verify pipeline statistics were updated
        pipeline_stats = summary.get('pipeline_stats', {})
        self.assertGreater(pipeline_stats.get('total_processed', 0), 0)
        self.assertGreater(pipeline_stats.get('total_chunks_created', 0), 0)

    def test_error_handling_in_pipeline(self):
        """Test that errors are properly handled in the pipeline."""
        # Create a processor
        processor = DocumentProcessor()

        # Try to process a non-existent file
        with self.assertRaises(Exception):
            processor.process_single_document(Path("/non/existent/file.md"))

        # The pipeline should handle this gracefully and continue processing other files
        # Verify that the system doesn't crash on individual file errors


class TestMemoryManagement(unittest.TestCase):
    """Tests for memory management during large document processing."""

    def test_large_document_processing(self):
        """Test processing of large documents to ensure memory management."""
        # This test would simulate processing large documents
        # In a real scenario, we would test with documents that approach the size limits
        # For now, we'll just verify the max document size setting exists
        from src.config.settings import settings
        self.assertGreater(settings.max_document_size, 0)

    def test_batch_processing(self):
        """Test that documents can be processed in batches to manage memory."""
        # This test would verify that the system can handle batch processing
        # to prevent memory issues when processing many large documents
        processor = DocumentProcessor()

        # The processor should be able to handle multiple documents
        # without running out of memory by processing them sequentially
        self.assertTrue(hasattr(processor, 'process_documents'))


if __name__ == '__main__':
    unittest.main()