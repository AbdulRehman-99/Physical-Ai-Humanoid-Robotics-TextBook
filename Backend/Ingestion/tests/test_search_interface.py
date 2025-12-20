"""Unit tests for the search interface module."""

import unittest
from unittest.mock import Mock, patch
from src.ingestion.search_interface import SearchInterface, SearchResult, create_search_engine


class TestSearchInterface(unittest.TestCase):
    """Test cases for SearchInterface class."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.search_interface = SearchInterface()

    @patch('src.ingestion.embedder.Embedder.embed_single_text')
    @patch('src.ingestion.vector_store.VectorStore.search_vectors')
    def test_search_basic(self, mock_search_vectors, mock_embed_single_text):
        """Test basic search functionality."""
        # Mock the embedding generation
        mock_embed_single_text.return_value = [0.1, 0.2, 0.3, 0.4]

        # Mock the search results
        mock_search_results = [
            {
                'id': 'chunk_1',
                'score': 0.95,
                'payload': {
                    'text_content': 'This is the first relevant result.',
                    'module': 'module1',
                    'chapter': 'chapter1',
                    'section': 'section1',
                    'source_file_path': 'test1.md'
                }
            },
            {
                'id': 'chunk_2',
                'score': 0.87,
                'payload': {
                    'text_content': 'This is the second relevant result.',
                    'module': 'module2',
                    'chapter': 'chapter2',
                    'section': 'section2',
                    'source_file_path': 'test2.md'
                }
            }
        ]
        mock_search_vectors.return_value = mock_search_results

        # Perform the search
        results = self.search_interface.search("test query", top_k=2)

        # Verify the results
        self.assertEqual(len(results), 2)
        self.assertIsInstance(results[0], SearchResult)
        self.assertEqual(results[0].chunk_id, 'chunk_1')
        self.assertEqual(results[0].text_content, 'This is the first relevant result.')
        self.assertEqual(results[0].score, 0.95)
        self.assertEqual(results[0].module, 'module1')
        self.assertEqual(results[0].chapter, 'chapter1')
        self.assertEqual(results[0].section, 'section1')
        self.assertEqual(results[0].source_file_path, 'test1.md')

        # Verify that the embedding and search methods were called
        mock_embed_single_text.assert_called_once_with("test query")
        mock_search_vectors.assert_called_once_with(
            query_vector=[0.1, 0.2, 0.3, 0.4],
            top_k=2,
            min_score=0.0
        )

    @patch('src.ingestion.embedder.Embedder.embed_single_text')
    @patch('src.ingestion.vector_store.VectorStore.search_vectors')
    def test_search_with_filters(self, mock_search_vectors, mock_embed_single_text):
        """Test search functionality with filters."""
        # Mock the embedding generation
        mock_embed_single_text.return_value = [0.1, 0.2, 0.3, 0.4]

        # Mock the search results
        mock_search_results = [
            {
                'id': 'chunk_1',
                'score': 0.95,
                'payload': {
                    'text_content': 'This is a filtered result.',
                    'module': 'module1',
                    'chapter': 'chapter1',
                    'section': 'section1',
                    'source_file_path': 'test1.md'
                }
            }
        ]
        mock_search_vectors.return_value = mock_search_results

        # Perform the search with filters
        filters = {'module': 'module1'}
        results = self.search_interface.search_with_filters(
            "test query",
            filters=filters,
            top_k=1,
            min_score=0.5
        )

        # Verify the results
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].chunk_id, 'chunk_1')
        self.assertEqual(results[0].text_content, 'This is a filtered result.')

        # Verify that the search method was called with filters
        mock_search_vectors.assert_called_once_with(
            query_vector=[0.1, 0.2, 0.3, 0.4],
            top_k=1,
            min_score=0.5,
            filters=filters
        )

    def test_format_search_results(self):
        """Test formatting search results."""
        results = [
            SearchResult(
                chunk_id='chunk_1',
                text_content='This is the first result content.',
                score=0.95,
                metadata={'key': 'value'},
                module='module1',
                chapter='chapter1',
                section='section1',
                source_file_path='test1.md'
            ),
            SearchResult(
                chunk_id='chunk_2',
                text_content='This is the second result content.',
                score=0.87,
                metadata={'key': 'value'},
                module='module2',
                chapter='chapter2',
                section='section2',
                source_file_path='test2.md'
            )
        ]

        formatted_output = self.search_interface.format_search_results(results)

        # Check that the formatted output contains expected elements
        self.assertIn("Result 1 (Score: 0.950)", formatted_output)
        self.assertIn("This is the first result content.", formatted_output)
        self.assertIn("Module 'module1'", formatted_output)
        self.assertIn("Chapter 'chapter1'", formatted_output)
        self.assertIn("Result 2 (Score: 0.870)", formatted_output)

    def test_format_search_results_no_citations(self):
        """Test formatting search results without citations."""
        results = [
            SearchResult(
                chunk_id='chunk_1',
                text_content='This is the first result content.',
                score=0.95,
                metadata={'key': 'value'},
                module='module1',
                chapter='chapter1',
                section='section1',
                source_file_path='test1.md'
            )
        ]

        formatted_output = self.search_interface.format_search_results(
            results,
            include_citations=False
        )

        # Check that the formatted output contains content but not citations
        self.assertIn("Result 1 (Score: 0.950)", formatted_output)
        self.assertIn("This is the first result content.", formatted_output)
        self.assertNotIn("Module 'module1'", formatted_output)
        self.assertNotIn("Chapter 'chapter1'", formatted_output)

    def test_format_search_results_truncated(self):
        """Test formatting search results with content truncation."""
        long_content = "A" * 200  # Create a long content string
        results = [
            SearchResult(
                chunk_id='chunk_1',
                text_content=long_content,
                score=0.95,
                metadata={'key': 'value'},
                module='module1',
                chapter='chapter1',
                section='section1',
                source_file_path='test1.md'
            )
        ]

        formatted_output = self.search_interface.format_search_results(
            results,
            max_content_length=50
        )

        # Check that the content was truncated
        self.assertIn("A" * 50, formatted_output)
        self.assertIn("...", formatted_output)
        self.assertNotIn(long_content, formatted_output)

    def test_validate_search_query_valid(self):
        """Test validating a valid search query."""
        issues = self.search_interface.validate_search_query("This is a valid query")
        self.assertEqual(len(issues), 0)

    def test_validate_search_query_empty(self):
        """Test validating an empty search query."""
        issues = self.search_interface.validate_search_query("")
        self.assertIn("Query is empty or contains only whitespace", issues)

    def test_validate_search_query_too_short(self):
        """Test validating a too short search query."""
        issues = self.search_interface.validate_search_query("hi")
        self.assertIn("Query is too short (minimum 3 characters)", issues)

    def test_validate_search_query_too_long(self):
        """Test validating a too long search query."""
        long_query = "A" * 600  # Create a query longer than 500 chars
        issues = self.search_interface.validate_search_query(long_query)
        self.assertIn("Query is too long (maximum 500 characters)", issues)

    def test_validate_search_query_harmful_content(self):
        """Test validating a query with potentially harmful content."""
        issues = self.search_interface.validate_search_query("DROP TABLE users")
        self.assertIn("Query contains potentially harmful keywords", issues)

    def test_create_search_engine(self):
        """Test creating a search engine instance."""
        engine = create_search_engine()
        self.assertIsInstance(engine, SearchInterface)


if __name__ == '__main__':
    unittest.main()