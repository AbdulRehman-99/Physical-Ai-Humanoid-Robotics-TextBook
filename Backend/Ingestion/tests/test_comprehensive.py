"""Comprehensive test suite for all components of the book ingestion system."""

import unittest
import sys
from pathlib import Path

# Add the src directory to the path so we can import modules
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))

# Import all test modules
from test_text_cleaner import TestTextCleaner
from test_chunker import TestChunker
from test_document_processor import TestDocumentProcessor
from test_search_interface import TestSearchInterface
from test_rag_integration import TestRAGIntegration, TestRAGPerformance


def create_comprehensive_test_suite():
    """Create a comprehensive test suite that includes all test cases."""

    # Create a test suite
    suite = unittest.TestSuite()

    # Add all test cases from different modules
    loader = unittest.TestLoader()

    # Add text cleaner tests
    suite.addTests(loader.loadTestsFromTestCase(TestTextCleaner))

    # Add chunker tests
    suite.addTests(loader.loadTestsFromTestCase(TestChunker))

    # Add document processor tests
    suite.addTests(loader.loadTestsFromTestCase(TestDocumentProcessor))

    # Add search interface tests
    suite.addTests(loader.loadTestsFromTestCase(TestSearchInterface))

    # Add RAG integration tests
    suite.addTests(loader.loadTestsFromTestCase(TestRAGIntegration))
    suite.addTests(loader.loadTestsFromTestCase(TestRAGPerformance))

    return suite


def run_comprehensive_tests():
    """Run the comprehensive test suite."""
    suite = create_comprehensive_test_suite()

    # Create a test runner
    runner = unittest.TextTestRunner(verbosity=2)

    # Run the tests
    result = runner.run(suite)

    # Return the result for further processing if needed
    return result


if __name__ == '__main__':
    print("Running comprehensive test suite for book ingestion system...")
    print("=" * 60)

    result = run_comprehensive_tests()

    print("=" * 60)
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success rate: {((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100):.2f}%")

    if result.failures:
        print("\nFailures:")
        for test, traceback in result.failures:
            print(f"  {test}: {traceback}")

    if result.errors:
        print("\nErrors:")
        for test, traceback in result.errors:
            print(f"  {test}: {traceback}")

    # Exit with appropriate code
    sys.exit(0 if result.wasSuccessful() else 1)