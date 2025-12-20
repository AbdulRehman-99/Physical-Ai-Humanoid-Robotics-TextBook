import pytest
from unittest.mock import Mock
from src.retrieval.validation import ValidationService
from src.retrieval.models import RetrievalResult, ValidationResult


class TestValidationContract:
    """Contract tests for validation function."""

    def test_validation_service_exists(self):
        """Test that the validation service exists."""
        validator = ValidationService()
        assert validator is not None

    def test_validation_service_has_required_methods(self):
        """Test that validation service has required methods."""
        validator = ValidationService()
        required_methods = ['validate_relevance', 'validate_metadata', 'validate_content_relevance']
        for method in required_methods:
            assert hasattr(validator, method)
            assert callable(getattr(validator, method))


class TestMetadataValidation:
    """Unit tests for metadata validation."""

    def test_metadata_validation_with_correct_metadata(self):
        """Test metadata validation with correct metadata."""
        validator = ValidationService()
        result = RetrievalResult(
            id="test-result-1",
            chunk_id="chunk-001",
            text="Sample text content",
            similarity_score=0.85,
            metadata={"module": "module1", "chapter": "ch1", "glossary": False},
            position=1
        )

        is_valid = validator.validate_metadata(result)
        assert is_valid is True

    def test_metadata_validation_with_missing_metadata(self):
        """Test metadata validation with missing metadata."""
        validator = ValidationService()
        result = RetrievalResult(
            id="test-result-1",
            chunk_id="chunk-001",
            text="Sample text content",
            similarity_score=0.85,
            metadata={},
            position=1
        )

        is_valid = validator.validate_metadata(result)
        assert is_valid is False

    def test_metadata_validation_with_incorrect_metadata_format(self):
        """Test metadata validation with incorrect metadata format."""
        validator = ValidationService()
        result = RetrievalResult(
            id="test-result-1",
            chunk_id="chunk-001",
            text="Sample text content",
            similarity_score=0.85,
            metadata={"module": 123, "chapter": None, "glossary": "false"},  # Wrong types
            position=1
        )

        is_valid = validator.validate_metadata(result)
        assert is_valid is False


class TestRelevanceValidation:
    """Unit tests for relevance validation."""

    def test_relevance_validation_with_high_similarity(self):
        """Test relevance validation with high similarity score."""
        validator = ValidationService()
        result = RetrievalResult(
            id="test-result-1",
            chunk_id="chunk-001",
            text="Sample text content",
            similarity_score=0.85,  # Above threshold of 0.7
            metadata={"module": "module1", "chapter": "ch1", "glossary": False},
            position=1
        )

        is_valid = validator.validate_relevance(result)
        assert is_valid is True

    def test_relevance_validation_with_low_similarity(self):
        """Test relevance validation with low similarity score."""
        validator = ValidationService()
        result = RetrievalResult(
            id="test-result-1",
            chunk_id="chunk-001",
            text="Sample text content",
            similarity_score=0.5,  # Below threshold of 0.7
            metadata={"module": "module1", "chapter": "ch1", "glossary": False},
            position=1
        )

        is_valid = validator.validate_relevance(result)
        assert is_valid is False

    def test_relevance_validation_with_exact_threshold(self):
        """Test relevance validation with exact threshold value."""
        validator = ValidationService()
        result = RetrievalResult(
            id="test-result-1",
            chunk_id="chunk-001",
            text="Sample text content",
            similarity_score=0.7,  # Exact threshold
            metadata={"module": "module1", "chapter": "ch1", "glossary": False},
            position=1
        )

        is_valid = validator.validate_relevance(result)
        assert is_valid is True