from typing import List, Dict, Any, Optional
from .models import RetrievalResult, ValidationResult
from .logger import RetrievalLogger
from datetime import datetime


class ValidationService:
    """
    Implements validation functionality to verify the relevance of retrieved chunks
    and ensure metadata is correctly preserved with 99% accuracy requirement.
    """

    def __init__(self):
        self.logger = RetrievalLogger()
        self.min_similarity_score = 0.7  # Minimum similarity score requirement
        self.required_metadata_fields = ["module", "chapter", "glossary"]

    def validate_relevance(self, result: RetrievalResult) -> bool:
        """
        Implement relevance validation function with minimum 0.7 similarity score requirement.

        Args:
            result: The retrieval result to validate

        Returns:
            True if the result is relevant, False otherwise
        """
        is_relevant = result.similarity_score >= self.min_similarity_score

        validation_data = {
            "result_id": result.id,
            "similarity_score": result.similarity_score,
            "is_relevant": is_relevant,
            "threshold": self.min_similarity_score
        }
        self.logger.log_validation_result(validation_data)

        return is_relevant

    def validate_metadata(self, result: RetrievalResult) -> bool:
        """
        Implement metadata validation function with 99% accuracy requirement.

        Args:
            result: The retrieval result to validate

        Returns:
            True if the metadata is valid, False otherwise
        """
        # Check if all required metadata fields are present
        missing_fields = []
        for field in self.required_metadata_fields:
            if field not in result.metadata:
                missing_fields.append(field)

        # Check if metadata values are of correct types
        type_errors = []
        if "module" in result.metadata and not isinstance(result.metadata["module"], str):
            type_errors.append("module should be string")
        if "chapter" in result.metadata and not isinstance(result.metadata["chapter"], str):
            type_errors.append("chapter should be string")
        if "glossary" in result.metadata and not isinstance(result.metadata["glossary"], bool):
            type_errors.append("glossary should be boolean")

        is_valid = len(missing_fields) == 0 and len(type_errors) == 0

        validation_data = {
            "result_id": result.id,
            "metadata_accuracy": is_valid,
            "missing_fields": missing_fields,
            "type_errors": type_errors,
            "metadata": result.metadata
        }
        self.logger.log_validation_result(validation_data)

        return is_valid

    def validate_content_relevance(self, query: str, result: RetrievalResult) -> float:
        """
        Create validation utilities for content relevance with human validation integration.

        Args:
            query: The original query text
            result: The retrieval result to validate

        Returns:
            A relevance score between 0 and 1
        """
        # This is a simplified implementation
        # In a real system, this would involve more sophisticated NLP techniques
        # or human validation integration

        # Simple keyword overlap as a basic relevance measure
        query_words = set(query.lower().split())
        result_words = set(result.text.lower().split())

        overlap = len(query_words.intersection(result_words))
        total_unique_words = len(query_words.union(result_words))

        if total_unique_words == 0:
            relevance_score = 0.0
        else:
            relevance_score = overlap / total_unique_words

        # Adjust the score based on the similarity score from the retrieval
        # This combines content overlap with the semantic similarity
        combined_score = (relevance_score + result.similarity_score) / 2

        # Ensure the score is between 0 and 1
        relevance_score = max(0.0, min(1.0, combined_score))

        validation_data = {
            "result_id": result.id,
            "query": query,
            "content_relevance_score": relevance_score,
            "semantic_similarity": result.similarity_score
        }
        self.logger.log_validation_result(validation_data)

        return relevance_score

    def validate_metadata_accuracy(self, result: RetrievalResult) -> bool:
        """
        Add metadata accuracy verification (module, chapter, glossary).

        Args:
            result: The retrieval result to validate

        Returns:
            True if metadata accuracy is verified, False otherwise
        """
        # Check if module field follows expected pattern
        module_ok = True
        if "module" in result.metadata:
            module_value = result.metadata["module"]
            if not isinstance(module_value, str) or not module_value.startswith("module"):
                module_ok = False

        # Check if chapter field follows expected pattern
        chapter_ok = True
        if "chapter" in result.metadata:
            chapter_value = result.metadata["chapter"]
            if not isinstance(chapter_value, str) or not chapter_value.startswith("ch"):
                chapter_ok = False

        # Check if glossary field is boolean
        glossary_ok = True
        if "glossary" in result.metadata:
            glossary_value = result.metadata["glossary"]
            if not isinstance(glossary_value, bool):
                glossary_ok = False

        is_accurate = module_ok and chapter_ok and glossary_ok

        validation_data = {
            "result_id": result.id,
            "metadata_accuracy_verification": is_accurate,
            "module_check": module_ok,
            "chapter_check": chapter_ok,
            "glossary_check": glossary_ok
        }
        self.logger.log_validation_result(validation_data)

        return is_accurate

    def validate_result(self, query: str, result: RetrievalResult) -> ValidationResult:
        """
        Validate a single retrieval result for both relevance and metadata accuracy.

        Args:
            query: The original query text
            result: The retrieval result to validate

        Returns:
            ValidationResult containing validation information
        """
        relevance_valid = self.validate_relevance(result)
        metadata_valid = self.validate_metadata(result)
        content_relevance_score = self.validate_content_relevance(query, result)
        metadata_accuracy = self.validate_metadata_accuracy(result)

        # Overall validation: both relevance and metadata must be valid
        overall_valid = relevance_valid and metadata_valid

        validation_result = ValidationResult(
            result_id=result.id,
            relevance_score=content_relevance_score,
            metadata_accuracy=metadata_accuracy,
            validation_notes=f"Relevance: {relevance_valid}, Metadata: {metadata_valid}",
            validated_by="ValidationService",
            validated_at=datetime.now()
        )

        return validation_result

    def validate_results(self, query: str, results: List[RetrievalResult]) -> List[ValidationResult]:
        """
        Validate a list of retrieval results.

        Args:
            query: The original query text
            results: List of retrieval results to validate

        Returns:
            List of ValidationResult objects
        """
        validation_results = []
        for result in results:
            validation_result = self.validate_result(query, result)
            validation_results.append(validation_result)

        return validation_results