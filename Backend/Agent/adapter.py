from litellm import completion
from typing import Optional, Dict, Any
import os
import time
import logging
import settings


logger = logging.getLogger(__name__)


class GeminiAdapter:
    """
    Adapter to route OpenAI SDK requests to Google Gemini using LiteLLM
    """

    def __init__(self):
        # Configure LiteLLM for Gemini
        os.environ["GEMINI_API_KEY"] = settings.settings.gemini_api_key

    def create_client(self):
        """
        Create and return a LiteLLM client configured for Gemini
        """
        return self

    def chat_completions_create(
        self,
        model: str = None,
        messages: list = None,
        max_retries: int = 3,
        **kwargs
    ):
        """
        Wrapper for LiteLLM completion that mimics OpenAI's API
        Includes comprehensive error handling and retry logic
        """
        if model is None:
            model = settings.settings.gemini_model

        last_exception = None
        for attempt in range(max_retries):
            try:
                response = completion(
                    model=f"gemini/{model}",
                    messages=messages,
                    api_key=settings.settings.gemini_api_key,
                    **kwargs
                )
                return response
            except Exception as e:
                last_exception = e
                logger.warning(f"Gemini API call failed (attempt {attempt + 1}/{max_retries}): {str(e)}")

                if attempt < max_retries - 1:  # Don't sleep on the last attempt
                    time.sleep(2 ** attempt)  # Exponential backoff

        # If all retries failed, log the error and raise
        logger.error(f"All {max_retries} attempts to call Gemini API failed: {str(last_exception)}")
        raise last_exception

    def completions_create(
        self,
        model: str = None,
        prompt: str = None,
        max_retries: int = 3,
        **kwargs
    ):
        """
        Wrapper for LiteLLM text completion that mimics OpenAI's API
        Includes comprehensive error handling and retry logic
        """
        if model is None:
            model = settings.settings.gemini_model

        last_exception = None
        for attempt in range(max_retries):
            try:
                response = completion(
                    model=f"gemini/{model}",
                    prompt=prompt,
                    api_key=settings.settings.gemini_api_key,
                    **kwargs
                )
                return response
            except Exception as e:
                last_exception = e
                logger.warning(f"Gemini API call failed (attempt {attempt + 1}/{max_retries}): {str(e)}")

                if attempt < max_retries - 1:  # Don't sleep on the last attempt
                    time.sleep(2 ** attempt)  # Exponential backoff

        # If all retries failed, log the error and raise
        logger.error(f"All {max_retries} attempts to call Gemini API failed: {str(last_exception)}")
        raise last_exception


# Singleton instance
gemini_client = GeminiAdapter()


def get_gemini_client():
    """
    Get the configured Gemini client
    """
    return gemini_client