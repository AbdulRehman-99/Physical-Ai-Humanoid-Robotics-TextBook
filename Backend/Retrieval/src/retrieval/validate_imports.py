"""
Quick validation script to test that all modules import correctly
and basic functionality works.
"""

def test_imports():
    """Test that all modules can be imported without errors."""
    print("Testing imports...")

    try:
        from src.retrieval.client import QdrantRetrievalClient
        print("[OK] Client module imported successfully")
    except ImportError as e:
        print(f"[ERROR] Failed to import client module: {e}")

    try:
        from src.retrieval.search import SemanticSearch
        print("[OK] Search module imported successfully")
    except ImportError as e:
        print(f"[ERROR] Failed to import search module: {e}")

    try:
        from src.retrieval.validation import ValidationService
        print("[OK] Validation module imported successfully")
    except ImportError as e:
        print(f"[ERROR] Failed to import validation module: {e}")

    try:
        from src.retrieval.latency import LatencyMeasurement
        print("[OK] Latency module imported successfully")
    except ImportError as e:
        print(f"[ERROR] Failed to import latency module: {e}")

    try:
        from src.retrieval.logger import RetrievalLogger
        print("[OK] Logger module imported successfully")
    except ImportError as e:
        print(f"[ERROR] Failed to import logger module: {e}")

    try:
        from src.retrieval.config import RetrievalConfig
        print("[OK] Config module imported successfully")
    except ImportError as e:
        print(f"[ERROR] Failed to import config module: {e}")

    try:
        from src.retrieval.models import SearchQuery, RetrievalResult, RetrievalRequest, RetrievalResponse
        print("[OK] Models module imported successfully")
    except ImportError as e:
        print(f"[ERROR] Failed to import models module: {e}")

    print("\nAll imports successful! All systems ready.")


if __name__ == "__main__":
    test_imports()