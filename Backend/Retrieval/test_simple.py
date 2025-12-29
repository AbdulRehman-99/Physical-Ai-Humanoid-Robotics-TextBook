import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from src.retrieval.search import SemanticSearch
from src.retrieval.models import SearchQuery, RetrievalRequest
from datetime import datetime

# Test the basic functionality
try:
    print("Initializing search service...")
    search_service = SemanticSearch()
    print("Search service initialized successfully")
    print(f"Embedder initialized: {hasattr(search_service, 'embedder')}")

    print("Creating query...")
    query = SearchQuery(
        id="test-query",
        text="What is ROS 2?",
        type="general",
        created_at=datetime.now()
    )
    print("Query created successfully")

    print("Creating request...")
    request = RetrievalRequest(
        query=query,
        top_k=5,
        include_metadata=True
    )
    print("Request created successfully")

    print("Performing search...")
    response = search_service.search(request)
    print(f"Search completed. Response type: {type(response)}")
    print(f"Response has 'results' attribute: {hasattr(response, 'results')}")
    print(f"Response has 'execution_time_ms' attribute: {hasattr(response, 'execution_time_ms')}")
    print(f"Response query_id: {getattr(response, 'query_id', 'N/A')}")

    if hasattr(response, 'results'):
        print(f"Number of results: {len(response.results)}")
        for i, result in enumerate(response.results):
            print(f"  Result {i+1}: {result.text[:100]}..." if len(result.text) > 100 else f"  Result {i+1}: {result.text}")
            print(f"    Similarity score: {result.similarity_score}")
            print(f"    Metadata: {result.metadata}")
    else:
        print("No results attribute found in response")

except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()