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

    print("Creating query...")
    query = SearchQuery(
        id="test-query",
        text="What is ROS 2 navigation?",
        type="general",
        created_at=datetime.now()
    )
    print("Query created successfully")

    print("Creating request...")
    request = RetrievalRequest(
        query=query,
        top_k=10,
        include_metadata=True
    )
    print("Request created successfully")

    print("Performing search...")
    response = search_service.search(request)
    print(f"Search completed. Response type: {type(response)}")
    print(f"Response has 'results' attribute: {hasattr(response, 'results')}")
    print(f"Response has 'execution_time_ms' attribute: {hasattr(response, 'execution_time_ms')}")
    print(f"Response query_id: {response.query_id}")
    print(f"Number of results: {len(response.results) if hasattr(response, 'results') else 'N/A'}")

except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()