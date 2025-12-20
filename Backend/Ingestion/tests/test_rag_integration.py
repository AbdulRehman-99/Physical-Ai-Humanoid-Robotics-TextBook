"""Integration tests for the RAG search functionality."""

import unittest
import json
from unittest.mock import patch, Mock
from src.ingestion.rag_api import create_rag_api
from src.ingestion.search_interface import SearchInterface, SearchResult


class TestRAGIntegration(unittest.TestCase):
    """Integration tests for RAG API functionality."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.app = create_rag_api()
        self.app.config['TESTING'] = True
        self.client = self.app.test_client()

    @patch('src.ingestion.embedder.Embedder.embed_single_text')
    @patch('src.ingestion.vector_store.VectorStore.search_vectors')
    def test_search_endpoint(self, mock_search_vectors, mock_embed_single_text):
        """Test the search API endpoint."""
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
                    'source_file_path': 'test1.md',
                    'key': 'value'
                }
            }
        ]
        mock_search_vectors.return_value = mock_search_results

        # Make a request to the search endpoint
        response = self.client.post('/search',
                                    data=json.dumps({
                                        'query': 'test query',
                                        'top_k': 1,
                                        'min_score': 0.5
                                    }),
                                    content_type='application/json')

        # Verify the response
        self.assertEqual(response.status_code, 200)
        data = json.loads(response.data)
        self.assertIn('results', data)
        self.assertEqual(len(data['results']), 1)
        self.assertEqual(data['results'][0]['text_content'], 'This is the first relevant result.')
        self.assertEqual(data['total_results'], 1)

    @patch('src.ingestion.embedder.Embedder.embed_single_text')
    @patch('src.ingestion.vector_store.VectorStore.search_vectors')
    def test_formatted_search_endpoint(self, mock_search_vectors, mock_embed_single_text):
        """Test the formatted search API endpoint."""
        # Mock the embedding generation
        mock_embed_single_text.return_value = [0.1, 0.2, 0.3, 0.4]

        # Mock the search results
        mock_search_results = [
            SearchResult(
                chunk_id='chunk_1',
                text_content='This is the first relevant result.',
                score=0.95,
                metadata={'key': 'value'},
                module='module1',
                chapter='chapter1',
                section='section1',
                source_file_path='test1.md'
            )
        ]

        # Mock the search method to return the SearchResult objects
        with patch.object(SearchInterface, 'search_with_filters', return_value=mock_search_results):
            # Make a request to the formatted search endpoint
            response = self.client.post('/search/formatted',
                                        data=json.dumps({
                                            'query': 'test query',
                                            'top_k': 1,
                                            'include_citations': True
                                        }),
                                        content_type='application/json')

            # Verify the response
            self.assertEqual(response.status_code, 200)
            data = json.loads(response.data)
            self.assertIn('formatted_results', data)
            self.assertIn('Result 1', data['formatted_results'])
            self.assertIn('This is the first relevant result.', data['formatted_results'])
            self.assertIn('Module \'module1\'', data['formatted_results'])

    def test_health_endpoint(self):
        """Test the health check endpoint."""
        response = self.client.get('/health')
        self.assertEqual(response.status_code, 200)
        data = json.loads(response.data)
        self.assertEqual(data['status'], 'healthy')
        self.assertIn('RAG API is running', data['message'])

    def test_search_endpoint_missing_query(self):
        """Test the search endpoint with missing query."""
        response = self.client.post('/search',
                                    data=json.dumps({}),
                                    content_type='application/json')
        self.assertEqual(response.status_code, 400)
        data = json.loads(response.data)
        self.assertIn('error', data)

    @patch('src.ingestion.embedder.Embedder.embed_single_text')
    @patch('src.ingestion.vector_store.VectorStore.search_vectors')
    def test_search_endpoint_invalid_query(self, mock_search_vectors, mock_embed_single_text):
        """Test the search endpoint with invalid query."""
        # Don't mock embedding for this test to trigger validation
        response = self.client.post('/search',
                                    data=json.dumps({
                                        'query': 'hi',  # Too short
                                        'top_k': 1
                                    }),
                                    content_type='application/json')
        self.assertEqual(response.status_code, 400)
        data = json.loads(response.data)
        self.assertIn('error', data)
        self.assertIn('Invalid query', data['error'])

    @patch('src.ingestion.search_interface.SearchInterface.search_with_filters')
    def test_search_endpoint_internal_error(self, mock_search_with_filters):
        """Test the search endpoint with internal error."""
        mock_search_with_filters.side_effect = Exception("Test error")

        response = self.client.post('/search',
                                    data=json.dumps({
                                        'query': 'test query',
                                        'top_k': 1
                                    }),
                                    content_type='application/json')
        self.assertEqual(response.status_code, 500)
        data = json.loads(response.data)
        self.assertIn('error', data)


class TestRAGPerformance(unittest.TestCase):
    """Performance tests for RAG functionality."""

    def test_response_time_monitoring(self):
        """Test that response time monitoring is available."""
        search_interface = SearchInterface()
        monitoring_data = search_interface.get_response_time_monitoring()

        # Check that expected metrics are present
        expected_keys = [
            'avg_response_time',
            'last_response_time',
            'total_queries',
            'success_rate',
            'error_rate'
        ]

        for key in expected_keys:
            self.assertIn(key, monitoring_data)

        # Check that numeric values are reasonable
        self.assertIsInstance(monitoring_data['avg_response_time'], (int, float))
        self.assertGreaterEqual(monitoring_data['total_queries'], 0)
        self.assertGreaterEqual(monitoring_data['success_rate'], 0)
        self.assertLessEqual(monitoring_data['success_rate'], 1)


if __name__ == '__main__':
    unittest.main()