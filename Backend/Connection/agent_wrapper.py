import asyncio
import logging
from typing import Any, Dict
from .models import ChatContext
from .config import AGENT_TIMEOUT, AGENT_RETRY_ATTEMPTS
import os
import google.generativeai as genai

logger = logging.getLogger(__name__)

class AgentWrapper:
    def __init__(self):
        # Initialize Google Generative AI with Gemini
        self.api_key = os.getenv("GEMINI_API_KEY")
        self.model_name = os.getenv("GEMINI_MODEL", "gemini-2.5-flash")

        if self.api_key:
            genai.configure(api_key=self.api_key)
            self.model = genai.GenerativeModel(self.model_name)
        else:
            logger.error("GEMINI_API_KEY not found in environment variables")
            self.model = None

    async def process_with_context(self, context: ChatContext) -> str:
        """
        Process the context with the agent and return response
        """
        try:
            # Simulate agent processing with timeout and retry mechanisms
            response = await self._call_agent_with_retry(context)
            return response
        except Exception as e:
            logger.error(f"Error processing with agent: {str(e)}")
            # Return a polite response for academic reliability
            return "I can only answer questions about the book content. Please ask a question related to the book."

    async def _call_agent_with_retry(self, context: ChatContext) -> str:
        """
        Call agent with retry mechanism and timeout handling
        """
        last_error = None

        for attempt in range(AGENT_RETRY_ATTEMPTS):
            try:
                # Apply timeout to the agent call
                response = await asyncio.wait_for(
                    self._call_agent_internal(context),
                    timeout=AGENT_TIMEOUT
                )
                return response
            except asyncio.TimeoutError:
                logger.warning(f"Agent call timed out on attempt {attempt + 1}")
                last_error = "timeout"
                if attempt == AGENT_RETRY_ATTEMPTS - 1:
                    break
                await asyncio.sleep(2 ** attempt)  # Exponential backoff
            except Exception as e:
                logger.error(f"Agent call failed on attempt {attempt + 1}: {str(e)}")
                last_error = str(e)
                if attempt == AGENT_RETRY_ATTEMPTS - 1:
                    break
                await asyncio.sleep(2 ** attempt)  # Exponential backoff

        # If all retries failed, return a polite response
        if last_error == "timeout":
            return "The system is experiencing high load. Please try again later."
        else:
            return "I can only answer questions about the book content. Please ask a question related to the book."

    async def _call_agent_internal(self, context: ChatContext) -> str:
        """
        Internal method to call the agent - this calls the actual Gemini API
        """
        if not self.model:
            return "I can only answer questions about the book content. Please ask a question related to the book."

        try:
            # Check if this is an off-topic query
            off_topic_indicators = ["weather", "joke", "unrelated", "random", "movie", "sports", "food"]
            user_message_lower = context.user_message.lower()

            if any(indicator in user_message_lower for indicator in off_topic_indicators):
                # Politely refuse off-topic queries without returning HTTP errors
                return "I can only answer questions about the book content. Please ask a question related to the book."

            # Build the prompt based on context
            if context.selected_text:
                # If selected text is provided, use it as context
                prompt = f"""
                You are an AI assistant that must answer questions based ONLY on the provided book content.
                You have been given specific text from the book. Use ONLY this information to answer the question.
                Do not use any external knowledge or general information.

                Here is the book content you must use:
                {context.selected_text}

                Now answer this question using ONLY the above content:
                {context.user_message}

                If the provided content does not contain the answer, explicitly state that the information is not in the provided content.
                """
            elif context.retrieved_chunks:
                # If retrieved chunks are available, use them as context
                # retrieved_chunks is now a list of RetrievedChunk objects
                chunk_texts = [chunk.content for chunk in context.retrieved_chunks]
                combined_context = " ".join(chunk_texts)

                prompt = f"""
                You are an AI assistant that must answer questions based ONLY on the provided book content.
                You have been given specific text from the book. Use ONLY this information to answer the question.
                Do not use any external knowledge or general information.

                Here is the book content you must use:
                {combined_context[:2000]}  # Limit context to avoid exceeding token limits

                Now answer this question using ONLY the above content:
                {context.user_message}

                If the provided content does not contain the answer, explicitly state that the information is not in the provided content.
                """
            else:
                # If no context is available, ask the question directly but guide to focus on book content
                prompt = f"""
                You are an AI assistant that can only answer questions about book content.
                Do not use any external knowledge.
                Question: {context.user_message}

                Since no specific content was provided, explicitly state that you couldn't find relevant content in the book.
                """

            # Call the Gemini API asynchronously
            response = await asyncio.get_event_loop().run_in_executor(
                None,
                lambda: self.model.generate_content(prompt)
            )

            # Extract the text from the response
            if response and hasattr(response, 'text') and response.text:
                result_text = response.text.strip()
                logger.info(f"Gemini API returned: {result_text[:200]}...")  # Log first 200 chars for debugging
                return result_text
            else:
                logger.warning(f"Gemini API returned empty or invalid response: {response}")
                return f"I couldn't find relevant content in the book for your query: {context.user_message}. Please try rephrasing your question."

        except Exception as e:
            logger.error(f"Error calling Gemini API: {str(e)}")
            return "I can only answer questions about the book content. Please ask a question related to the book."