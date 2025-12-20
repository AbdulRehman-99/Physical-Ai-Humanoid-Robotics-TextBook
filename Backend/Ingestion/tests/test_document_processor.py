"""Unit tests for the document processor module."""

import unittest
from unittest.mock import Mock, patch, mock_open
from pathlib import Path
import tempfile
import os
from src.ingestion.document_processor import DocumentProcessor, create_ingestion_pipeline
from src.ingestion.models import DocumentMetadata
from src.ingestion.exceptions import DocumentProcessingError, ValidationError


class TestDocumentProcessor(unittest.TestCase):
    """Test cases for DocumentProcessor class."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.processor = DocumentProcessor()

    def test_extract_metadata_from_path(self):
        """Test extracting metadata from a file path."""
        # Create a mock path
        test_path = Path("Frontend/docs/module-1/chapter-1/introduction.md")

        metadata = self.processor.extract_metadata_from_path(test_path)

        self.assertIsInstance(metadata, DocumentMetadata)
        self.assertEqual(metadata.module, "module-1")
        self.assertEqual(metadata.chapter, "chapter-1")
        self.assertEqual(metadata.section, "introduction")
        self.assertEqual(metadata.file_name, "introduction.md")

    def test_read_and_parse_document_success(self):
        """Test successfully reading and parsing a markdown document."""
        test_content = """---
title: Test Document
author: Test Author
---
# Test Content

This is test content.
"""

        test_path = Path("test.md")

        with patch('builtins.open', mock_open(read_data=test_content)):
            with patch('os.path.exists', return_value=True):
                content, metadata = self.processor.read_and_parse_document(test_path)

                self.assertIn("Test Content", content)
                self.assertEqual(metadata.get('title'), 'Test Document')
                self.assertEqual(metadata.get('author'), 'Test Author')

    def test_read_and_parse_document_large_file(self):
        """Test that large files are rejected."""
        # Create a large content string (> 10MB default)
        large_content = "This is a test." * (1024 * 1024 * 2)  # ~20MB worth of text
        test_path = Path("large_test.md")

        with patch('builtins.open', mock_open(read_data=large_content)):
            with patch('src.ingestion.utils.get_file_size', return_value=20971520):  # 20MB
                with self.assertRaises(DocumentProcessingError):
                    self.processor.read_and_parse_document(test_path)

    def test_validate_document_content_success(self):
        """Test that valid content passes validation."""
        valid_content = "This is valid content."
        test_path = Path("test.md")

        result = self.processor.validate_document_content(valid_content, test_path)
        self.assertTrue(result)

    def test_validate_document_content_empty(self):
        """Test that empty content fails validation."""
        empty_content = ""
        test_path = Path("test.md")

        with self.assertRaises(ValidationError):
            self.processor.validate_document_content(empty_content, test_path)

    def test_validate_document_content_binary(self):
        """Test that binary content fails validation."""
        binary_content = "\x00" * 50 + "normal text"  # 50 null bytes in 55 char string > 50% threshold
        test_path = Path("test.md")

        with self.assertRaises(ValidationError):
            self.processor.validate_document_content(binary_content, test_path)

    @patch('src.ingestion.utils.get_all_markdown_files')
    def test_scan_documents(self, mock_get_files):
        """Test scanning for markdown files."""
        mock_files = [Path("doc1.md"), Path("doc2.md")]
        mock_get_files.return_value = mock_files

        result = self.processor.scan_documents("test_path")

        self.assertEqual(result, mock_files)
        mock_get_files.assert_called_once_with("test_path")


class TestIngestionPipeline(unittest.TestCase):
    """Test cases for the ingestion pipeline."""

    def test_create_ingestion_pipeline(self):
        """Test that the ingestion pipeline function is created properly."""
        pipeline_func = create_ingestion_pipeline()

        # The function should exist and be callable
        self.assertTrue(callable(pipeline_func))

        # It should have the expected signature (though we can't easily test defaults)
        import inspect
        sig = inspect.signature(pipeline_func)
        self.assertIn('source_path', sig.parameters)


if __name__ == '__main__':
    unittest.main()