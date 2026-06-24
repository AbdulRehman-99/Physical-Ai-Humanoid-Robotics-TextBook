import unittest
from src.ingestion.chunker import Chunker, Chunk


class TestChunker(unittest.TestCase):

    def setUp(self):
        self.chunker = Chunker(chunk_size=100, overlap_size=20)

    def test_create_chunks_small_text(self):
        text = "This is a short text."
        chunks = self.chunker.create_chunks(text)
        self.assertEqual(len(chunks), 1)
        self.assertEqual(chunks[0].text, "This is a short text.")
        self.assertEqual(chunks[0].index, 0)

    def test_create_chunks_empty_text(self):
        chunks = self.chunker.create_chunks("")
        self.assertEqual(chunks, [])

    def test_create_chunks_whitespace(self):
        chunks = self.chunker.create_chunks("   ")
        self.assertEqual(chunks, [])

    def test_create_chunks_respects_headings(self):
        text = "# Title\n\nSome content here.\n\n## Section 1\n\nThis is section one content.\n\n## Section 2\n\nThis is section two content."
        small_chunker = Chunker(chunk_size=5, overlap_size=1)
        chunks = small_chunker.create_chunks(text)
        self.assertGreater(len(chunks), 1)

    def test_chunk_document_content(self):
        from src.ingestion.models import DocumentChunk

        large_text = "First sentence. " * 50
        small_chunker = Chunker(chunk_size=20, overlap_size=5)
        doc_chunk = DocumentChunk(
            id="test_chunk",
            text_content=large_text,
            module="test_module",
            chapter="test_chapter",
            section="test_section",
            book_version="v1.0",
            source_file_path="test.md",
            chunk_index=0,
        )

        result_chunks = small_chunker.chunk_document_content([doc_chunk])
        self.assertGreater(len(result_chunks), 1)

    def test_validate_chunk_quality_good(self):
        chunk = Chunk(
            id="test_chunk",
            text="This is a good quality text chunk with meaningful content.",
            index=0,
            metadata={},
        )
        is_ok, issues = self.chunker.validate_chunk_quality(chunk)
        self.assertTrue(is_ok)
        self.assertEqual(len(issues), 0)

    def test_validate_chunk_quality_poor_content(self):
        chunk = Chunk(
            id="test_chunk",
            text="$$$",
            index=0,
            metadata={},
        )
        is_ok, issues = self.chunker.validate_chunk_quality(chunk)
        self.assertFalse(is_ok)
        self.assertGreater(len(issues), 0)

    def test_validate_chunk_quality_high_special_chars(self):
        chunk = Chunk(
            id="test_chunk",
            text="$$$$$$$$$$ @@@@@@@@@@ $$$$$$$$$$ special $$$$$$$$$$",
            index=0,
            metadata={},
        )
        is_ok, issues = self.chunker.validate_chunk_quality(chunk)
        self.assertFalse(is_ok)
        self.assertTrue(any("special characters" in issue for issue in issues))


if __name__ == '__main__':
    unittest.main()
