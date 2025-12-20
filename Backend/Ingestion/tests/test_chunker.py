"""Unit tests for the chunker module."""

import unittest
from src.ingestion.chunker import Chunker, Chunk


class TestChunker(unittest.TestCase):
    """Test cases for Chunker class."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.chunker = Chunker(chunk_size=100, overlap_size=20)

    def test_tokenize_simple(self):
        """Test tokenization of simple text."""
        text = "This is a test sentence. This is another sentence."
        tokens = self.chunker.tokenize(text)
        expected_tokens = ['This', 'is', 'a', 'test', 'sentence.', '<EOS>', 'This', 'is', 'another', 'sentence.', '<EOS>']
        self.assertEqual(tokens, expected_tokens)

    def test_detokenize_simple(self):
        """Test detokenization of simple tokens."""
        tokens = ['This', 'is', 'a', 'test', 'sentence.', '<EOS>', 'This', 'is', 'another', 'sentence.', '<EOS>']
        text = self.chunker.detokenize(tokens)
        expected_text = "This is a test sentence. This is another sentence."
        self.assertEqual(text, expected_text)

    def test_identify_semantic_boundaries(self):
        """Test identifying semantic boundaries in tokens."""
        tokens = ['This', 'is', 'the', 'first', 'sentence.', '<EOS>', 'This', 'is', 'the', 'second', 'sentence.', '<EOS>', 'And', 'this', 'is', 'the', 'third.']
        boundaries = self.chunker.identify_semantic_boundaries(tokens)
        expected_boundaries = [5, 11]  # Indices of <EOS> tokens
        self.assertEqual(boundaries, expected_boundaries)

    def test_create_semantic_chunks_small_text(self):
        """Test creating semantic chunks for text smaller than chunk size."""
        text = "This is a short text."
        chunks = self.chunker.create_semantic_chunks(text)
        self.assertEqual(len(chunks), 1)
        self.assertEqual(chunks[0].text, "This is a short text.")
        self.assertEqual(chunks[0].index, 0)

    def test_create_semantic_chunks_with_sentence_boundaries(self):
        """Test creating semantic chunks that respect sentence boundaries."""
        text = "First sentence. Second sentence. Third sentence. Fourth sentence."
        # Set a small chunk size to force chunking
        small_chunker = Chunker(chunk_size=6, overlap_size=1)
        chunks = small_chunker.create_semantic_chunks(text)

        # Should create multiple chunks respecting sentence boundaries where possible
        self.assertGreater(len(chunks), 1)

        # Check that chunks contain complete sentences where possible
        for chunk in chunks:
            self.assertIn('.', chunk.text)  # Each chunk should contain complete sentences

    def test_chunk_document_content(self):
        """Test chunking of DocumentChunk objects."""
        from src.ingestion.models import DocumentChunk

        # Create a large text that will need chunking
        large_text = "First sentence. " * 50  # Create a text longer than default chunk size
        doc_chunk = DocumentChunk(
            id="test_chunk",
            text_content=large_text,
            module="test_module",
            chapter="test_chapter",
            section="test_section",
            book_version="v1.0",
            source_file_path="test.md",
            chunk_index=0
        )

        # Process the chunk
        result_chunks = self.chunker.chunk_document_content([doc_chunk])

        # Should have created more than one chunk
        self.assertGreater(len(result_chunks), 1)

        # Check that the original content is preserved across chunks
        combined_text = " ".join([chunk.text_content for chunk in result_chunks])
        original_tokens = self.chunker.tokenize(large_text)
        combined_tokens = self.chunker.tokenize(combined_text)

        # The token count should be similar (might differ slightly due to processing)
        self.assertAlmostEqual(len(original_tokens), len(combined_tokens), delta=5)

    def test_validate_chunk_parameters_valid(self):
        """Test validating valid chunk parameters."""
        is_valid = self.chunker.validate_chunk_parameters(512, 100)
        self.assertTrue(is_valid)

    def test_validate_chunk_parameters_invalid_size(self):
        """Test validating invalid chunk size."""
        is_valid = self.chunker.validate_chunk_parameters(-100, 50)
        self.assertFalse(is_valid)

    def test_validate_chunk_parameters_invalid_overlap(self):
        """Test validating invalid overlap size."""
        is_valid = self.chunker.validate_chunk_parameters(100, -10)
        self.assertFalse(is_valid)

    def test_validate_chunk_parameters_overlap_greater_than_size(self):
        """Test validating overlap greater than chunk size."""
        is_valid = self.chunker.validate_chunk_parameters(100, 150)
        self.assertFalse(is_valid)

    def test_validate_chunk_quality_good(self):
        """Test validating a good quality chunk."""
        chunk = Chunk(
            id="test_chunk",
            text="This is a good quality text chunk with meaningful content.",
            index=0,
            metadata={}
        )
        is_ok, issues = self.chunker.validate_chunk_quality(chunk)
        self.assertTrue(is_ok)
        self.assertEqual(len(issues), 0)

    def test_validate_chunk_quality_poor_content(self):
        """Test validating a chunk with poor content."""
        chunk = Chunk(
            id="test_chunk",
            text="$$$",  # Very little meaningful content
            index=0,
            metadata={}
        )
        is_ok, issues = self.chunker.validate_chunk_quality(chunk)
        self.assertFalse(is_ok)
        self.assertGreater(len(issues), 0)

    def test_validate_chunk_quality_high_special_chars(self):
        """Test validating a chunk with high ratio of special characters."""
        chunk = Chunk(
            id="test_chunk",
            text="This text has too many $$$ and @@@ and !!! special characters",
            index=0,
            metadata={}
        )
        is_ok, issues = self.chunker.validate_chunk_quality(chunk)
        self.assertFalse(is_ok)
        # Should identify high ratio of special characters
        self.assertTrue(any("special characters" in issue for issue in issues))


if __name__ == '__main__':
    unittest.main()