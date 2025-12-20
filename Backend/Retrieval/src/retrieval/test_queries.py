"""
Sample test queries for retrieval testing scripts.
"""

from typing import List, Dict, Any


def get_general_queries() -> List[str]:
    """
    Get a list of general queries for testing semantic similarity search.

    Returns:
        List of general query strings
    """
    return [
        "What is ROS 2 navigation?",
        "Explain PID controllers in robotics",
        "How does computer vision work in robotics?",
        "What are the fundamentals of ROS?",
        "Describe the architecture of ROS 2",
        "How do sensors work in ROS?",
        "What is the difference between ROS 1 and ROS 2?",
        "Explain the concept of nodes in ROS",
        "How does message passing work in ROS?",
        "What are ROS packages and how to create them?"
    ]


def get_selected_text_queries() -> List[str]:
    """
    Get a list of selected text queries for testing exact match + semantic similarity.

    Returns:
        List of selected text query strings
    """
    return [
        "selected text for testing",
        "exact match query example",
        "specific phrase to match",
        "particular text fragment",
        "targeted text selection"
    ]


def get_validation_queries() -> List[Dict[str, Any]]:
    """
    Get queries with expected validation results for testing validation functionality.

    Returns:
        List of query dictionaries with expected results
    """
    return [
        {
            "query": "What is ROS 2 navigation?",
            "expected_module": "module-4-advanced-integration",
            "expected_chapter": "ch7-whisper-llm-vla-planning",
            "min_similarity": 0.7
        },
        {
            "query": "Explain PID controllers",
            "expected_module": "module-3-nvidia-isaac",
            "expected_chapter": "ch6-isaac-ros-vslam-localization-nav2",
            "min_similarity": 0.6
        },
        {
            "query": "How does Gazebo simulation work?",
            "expected_module": "module-2-simulation",
            "expected_chapter": "ch4-unity-high-fidelity-interaction",
            "min_similarity": 0.65
        }
    ]


def get_performance_queries() -> List[str]:
    """
    Get queries for performance testing.

    Returns:
        List of query strings for performance testing
    """
    return [
        "performance test query 1",
        "load test query 2",
        "stress test query 3",
        "concurrency test query 4",
        "latency measurement query 5"
    ]


def get_all_test_queries() -> Dict[str, List]:
    """
    Get all test queries organized by type.

    Returns:
        Dictionary with query types as keys and lists of queries as values
    """
    return {
        "general": get_general_queries(),
        "selected_text": get_selected_text_queries(),
        "validation": [q["query"] for q in get_validation_queries()],
        "performance": get_performance_queries()
    }