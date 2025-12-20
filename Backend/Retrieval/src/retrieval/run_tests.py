#!/usr/bin/env python3
"""
Main retrieval test script that integrates all user stories for comprehensive retrieval testing.
"""

import argparse
import sys
from datetime import datetime
from typing import List, Dict, Any

from .search import SemanticSearch
from .validation import ValidationService
from .latency import LatencyMeasurement
from .models import SearchQuery, RetrievalRequest
from .logger import RetrievalLogger


def create_sample_queries() -> List[Dict[str, Any]]:
    """
    Create sample test queries for comprehensive retrieval testing.

    Returns:
        List of query dictionaries with different types and parameters
    """
    return [
        {
            "text": "What is ROS 2 navigation?",
            "type": "general",
            "expected_module": "module-4-advanced-integration"
        },
        {
            "text": "Explain PID controllers in robotics",
            "type": "general",
            "expected_module": "module-3-nvidia-isaac"
        },
        {
            "text": "How does computer vision work in robotics?",
            "type": "general",
            "expected_module": "module-2-simulation"
        },
        {
            "text": "What are the fundamentals of ROS?",
            "type": "general",
            "expected_module": "module-1-ros"
        },
        {
            "text": "selected text for testing exact match",
            "type": "selected-text",
            "expected_module": "module-1-ros"
        }
    ]


def run_comprehensive_tests():
    """
    Run comprehensive retrieval tests integrating all user stories.
    """
    print("Starting comprehensive retrieval tests...")
    print(f"Test run started at: {datetime.now().isoformat()}")
    print("-" * 60)

    # Initialize components
    search_service = SemanticSearch()
    validation_service = ValidationService()
    latency_service = LatencyMeasurement()
    logger = RetrievalLogger()

    # Create sample queries
    sample_queries = create_sample_queries()
    all_results = []

    print(f"Running {len(sample_queries)} test queries...\n")

    for i, query_data in enumerate(sample_queries, 1):
        print(f"Test {i}: {query_data['text']}")
        print(f"  Query type: {query_data['type']}")

        # Create search query object
        search_query = SearchQuery(
            id=f"test-query-{i}",
            text=query_data["text"],
            type=query_data["type"],
            created_at=datetime.now(),
            metadata_filters=None
        )

        # Create retrieval request
        request = RetrievalRequest(
            query=search_query,
            top_k=10,
            include_metadata=True,
            exact_match_first=(query_data["type"] == "selected-text")
        )

        # Perform search
        response = search_service.search(request)

        print(f"  Retrieved {len(response.results)} results in {response.execution_time_ms:.2f}ms")

        # Validate results
        validation_results = validation_service.validate_results(query_data["text"], response.results)
        valid_results_count = sum(1 for vr in validation_results if vr.metadata_accuracy and vr.relevance_score >= 0.7)
        print(f"  {valid_results_count}/{len(validation_results)} results passed validation")

        # Measure performance - create a function that matches expected interface
        def search_with_params():
            temp_query = SearchQuery(
                id="perf-query",
                text=query_data["text"],
                type=query_data["type"],
                created_at=datetime.now()
            )
            temp_request = RetrievalRequest(
                query=temp_query,
                top_k=request.top_k,
                include_metadata=True
            )
            return search_service.search(temp_request)

        perf_data = latency_service.add_performance_tracking_to_search(
            search_with_params, query_data["text"], request.top_k
        )
        print(f"  Performance: {perf_data['execution_time_ms']:.2f}ms")

        # Log results
        logger.log_retrieval_response({
            "query_id": response.query_id,
            "num_results": len(response.results),
            "execution_time_ms": response.execution_time_ms,
            "valid_results_count": valid_results_count
        })

        # Store for summary
        all_results.append({
            "query": query_data,
            "response": response,
            "validation_results": validation_results,
            "performance": perf_data
        })

        print()

    # Run performance tests
    print("Running performance and stability tests...")
    perf_result = latency_service.measure_latency(
        lambda q, k=10: search_service.search(
            RetrievalRequest(
                query=SearchQuery(
                    id="perf-test",
                    text=q,
                    type="general",
                    created_at=datetime.now()
                ),
                top_k=k
            )
        ),
        "performance test query",
        iterations=10
    )
    print(f"  Average latency: {perf_result.latency_ms:.2f}ms")
    print(f"  Throughput: {perf_result.throughput_qps:.2f} QPS")
    print(f"  Stability: {perf_result.stability_score:.2f}")

    # Summary
    total_results = sum(len(r["response"].results) for r in all_results)
    total_valid = sum(
        sum(1 for vr in r["validation_results"] if vr.metadata_accuracy and vr.relevance_score >= 0.7)
        for r in all_results
    )

    print("-" * 60)
    print("TEST SUMMARY:")
    print(f"  Total queries: {len(sample_queries)}")
    print(f"  Total results retrieved: {total_results}")
    print(f"  Valid results: {total_valid}")
    print(f"  Success rate: {(total_valid/total_results*100) if total_results > 0 else 0:.1f}%")
    print(f"  Test run completed at: {datetime.now().isoformat()}")


def main():
    """
    Main function to run the retrieval tests.
    """
    parser = argparse.ArgumentParser(description="Run comprehensive retrieval tests")
    parser.add_argument("--queries", "-q", type=str, nargs="*",
                        help="Specific queries to test (optional, defaults to sample queries)")
    parser.add_argument("--top-k", "-k", type=int, default=10,
                        help="Number of results to retrieve (default: 10)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Enable verbose output")

    args = parser.parse_args()

    try:
        run_comprehensive_tests()
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
        sys.exit(1)
    except Exception as e:
        print(f"Error running tests: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()