"""API endpoints for RAG chatbot integration."""

from flask import Flask, request, jsonify
from typing import Dict, Any, Optional
import logging
import time
from src.ingestion.search_interface import SearchInterface
from src.config.settings import settings


def create_rag_api():
    """Create a Flask app with RAG API endpoints."""
    app = Flask(__name__)

    # Set up logging
    logging.basicConfig(level=getattr(logging, settings.log_level.upper()))
    logger = logging.getLogger(__name__)

    # Initialize the search interface
    search_interface = SearchInterface()

    @app.route('/health', methods=['GET'])
    def health():
        try:
            return jsonify({'status': 'healthy', 'message': 'RAG API is running'}), 200
        except Exception as e:
            logger.error(f"Health check failed: {e}")
            return jsonify({'status': 'unhealthy', 'error': str(e)}), 500

    @app.route('/search', methods=['POST'])
    def search():
        """Perform semantic search against the vector database."""
        start_time = time.time()
        logger.info("Received search request")

        try:
            # Get request data
            data = request.get_json()

            if not data or 'query' not in data:
                return jsonify({'error': 'Missing query in request body'}), 400

            query = data.get('query')
            top_k = data.get('top_k', 5)
            min_score = data.get('min_score', 0.0)
            filters = data.get('filters', {})
            include_citations = data.get('include_citations', True)
            max_content_length = data.get('max_content_length', None)

            # Validate query
            validation_issues = search_interface.validate_search_query(query)
            if validation_issues:
                return jsonify({'error': 'Invalid query', 'issues': validation_issues}), 400

            # Perform search
            results = search_interface.search_with_filters(
                query=query,
                filters=filters,
                top_k=top_k,
                min_score=min_score
            )

            # Format results
            formatted_results = []
            for result in results:
                formatted_result = {
                    'chunk_id': result.chunk_id,
                    'text_content': result.text_content,
                    'score': result.score,
                    'metadata': result.metadata,
                    'module': result.module,
                    'chapter': result.chapter,
                    'section': result.section,
                    'source_file_path': result.source_file_path
                }
                formatted_results.append(formatted_result)

            # Calculate response time
            response_time = time.time() - start_time

            # Add response time monitoring
            response_data = {
                'query': query,
                'results': formatted_results,
                'total_results': len(formatted_results),
                'response_time_seconds': response_time,
                'search_parameters': {
                    'top_k': top_k,
                    'min_score': min_score,
                    'filters': filters
                }
            }

            logger.info(f"Search completed successfully in {response_time:.3f}s, returned {len(formatted_results)} results")
            return jsonify(response_data), 200

        except Exception as e:
            logger.error(f"Search failed: {e}")
            logger.exception("Full traceback:")
            return jsonify({'error': f'Search failed: {str(e)}'}), 500

    @app.route('/search/formatted', methods=['POST'])
    def search_formatted():
        """Perform semantic search and return formatted results for direct use in chatbot."""
        start_time = time.time()
        logger.info("Received formatted search request")

        try:
            # Get request data
            data = request.get_json()

            if not data or 'query' not in data:
                return jsonify({'error': 'Missing query in request body'}), 400

            query = data.get('query')
            top_k = data.get('top_k', 5)
            min_score = data.get('min_score', 0.0)
            filters = data.get('filters', {})
            include_citations = data.get('include_citations', True)
            max_content_length = data.get('max_content_length', None)

            # Validate query
            validation_issues = search_interface.validate_search_query(query)
            if validation_issues:
                return jsonify({'error': 'Invalid query', 'issues': validation_issues}), 400

            # Perform search
            results = search_interface.search_with_filters(
                query=query,
                filters=filters,
                top_k=top_k,
                min_score=min_score
            )

            # Format results as a single formatted string
            formatted_output = search_interface.format_search_results(
                results,
                include_citations=include_citations,
                max_content_length=max_content_length
            )

            # Calculate response time
            response_time = time.time() - start_time

            response_data = {
                'query': query,
                'formatted_results': formatted_output,
                'total_results': len(results),
                'response_time_seconds': response_time
            }

            logger.info(f"Formatted search completed successfully in {response_time:.3f}s")
            return jsonify(response_data), 200

        except Exception as e:
            logger.error(f"Formatted search failed: {e}")
            logger.exception("Full traceback:")
            return jsonify({'error': f'Formatted search failed: {str(e)}'}), 500

    return app


if __name__ == '__main__':
    # For development/testing purposes
    app = create_rag_api()
    app.run(host='0.0.0.0', port=5000, debug=True)