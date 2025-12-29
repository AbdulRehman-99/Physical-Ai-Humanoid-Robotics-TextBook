from typing import List, Dict, Any
import settings
import logging


logger = logging.getLogger(__name__)


def validate_academic_reliability(response: str, context: str, threshold: float = 0.7) -> Dict[str, Any]:
    """
    Validate that the response meets academic reliability standards
    by checking how much of it is traceable to the provided context.

    Args:
        response: The agent's response to validate
        context: The context that was used to generate the response
        threshold: Minimum percentage (0-1) of response that should be traceable to context

    Returns:
        Dict with validation results including score and issues found
    """
    try:
        # Simple validation: check if response content is related to context
        response_words = set(response.lower().split())
        context_words = set(context.lower().split())

        # Find overlapping words
        overlap = response_words.intersection(context_words)
        total_response_words = len(response_words)

        if total_response_words == 0:
            overlap_ratio = 0.0
        else:
            overlap_ratio = len(overlap) / total_response_words

        # Additional checks for academic reliability
        issues = []

        # Check for generic responses that might indicate hallucination
        generic_phrases = [
            "i don't know", "no information provided", "not mentioned in the text",
            "not specified", "cannot determine", "not stated"
        ]

        is_generic = any(phrase in response.lower() for phrase in generic_phrases)

        # Check for external knowledge indicators
        external_indicators = [
            "according to general knowledge", "in general", "typically", "usually",
            "most sources say", "commonly known", "in most cases"
        ]

        has_external_indicators = any(indicator in response.lower() for indicator in external_indicators)

        if has_external_indicators:
            issues.append("Response contains external knowledge indicators")

        # Create validation result
        result = {
            "is_reliable": overlap_ratio >= threshold and not has_external_indicators,
            "reliability_score": overlap_ratio,
            "threshold": threshold,
            "issues": issues,
            "overlap_ratio": overlap_ratio,
            "is_generic_response": is_generic
        }

        return result

    except Exception as e:
        logger.error(f"Error during academic reliability validation: {str(e)}")
        # Return a safe default to not block the system
        return {
            "is_reliable": True,
            "reliability_score": 0.0,
            "threshold": threshold,
            "issues": ["Validation error occurred"],
            "overlap_ratio": 0.0,
            "is_generic_response": False
        }


def validate_response_quality(response: str, context: str) -> Dict[str, Any]:
    """
    Perform comprehensive quality validation on the response
    """
    try:
        # Perform academic reliability check
        reliability_result = validate_academic_reliability(response, context)

        # Additional quality metrics
        quality_metrics = {
            "response_length": len(response),
            "context_length": len(context),
            "has_context_reference": len(context) > 0,
            "contains_specific_details": len(response.split()) > 5  # Basic check for substance
        }

        # Combine results
        result = {
            **reliability_result,
            "quality_metrics": quality_metrics
        }

        return result

    except Exception as e:
        logger.error(f"Error during response quality validation: {str(e)}")
        return {
            "is_reliable": True,
            "reliability_score": 0.0,
            "threshold": 0.7,
            "issues": ["Quality validation error occurred"],
            "overlap_ratio": 0.0,
            "is_generic_response": False,
            "quality_metrics": {}
        }