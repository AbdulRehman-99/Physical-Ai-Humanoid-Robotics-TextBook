from typing import Optional, List, Dict, Any
from pydantic import BaseModel
from adapter import get_gemini_client
import sys
import os
import importlib.util
from pathlib import Path

# Add the Retrieval/src directory to the path
# Since this file is in Backend/Agent and Retrieval is in Backend/Retrieval
retrieval_src_path = Path(__file__).parent.parent / "Retrieval" / "src"
sys.path.insert(0, str(retrieval_src_path))

from retrieval import get_retrieval_service
import settings
from validation import validate_response_quality
import logging
import asyncio
import time


logger = logging.getLogger(__name__)


class AgentResponse(BaseModel):
    """
    Response model for the agent
    """
    response: str
    source_chunks: List[str]
    confidence: float


class ContextSource(BaseModel):
    """
    Model for tracking the source of information for the agent
    """
    type: str  # "selected_text", "vector_retrieval", or "llm_only"
    content: str
    retrieved_chunks_count: int = 0
    metadata: Dict[str, Any] = {}


class RAGAgent:
    """
    RAG Agent class using OpenAI Agent SDK v0.6 pattern
    """

    def __init__(self):
        self.gemini_client = get_gemini_client()
        self.retrieval_service = get_retrieval_service()

    async def process_message(self, message: str, selected_text: Optional[str] = None) -> AgentResponse:
        """
        Process a user message and return an appropriate response
        """
        try:
            # Check if the message is off-topic (not related to book content)
            if not selected_text or selected_text.strip() == "":
                is_off_topic = await self.check_off_topic(message)
                if is_off_topic:
                    return AgentResponse(
                        response="Ask the question related to the book",
                        source_chunks=[],
                        confidence=0.5
                    )

            # Determine context source based on selected_text
            if selected_text is not None and selected_text.strip() != "":
                # Use selected text as context
                context_source = ContextSource(
                    type="selected_text",
                    content=selected_text,
                    retrieved_chunks_count=0,
                    metadata={"origin": "user_selection"}
                )
            else:
                # Use vector retrieval for context
                try:
                    retrieved_context = self.retrieval_service.get_book_context(message)
                    context_source = ContextSource(
                        type="vector_retrieval",
                        content=retrieved_context,
                        retrieved_chunks_count=len(retrieved_context.split()) if retrieved_context else 0,
                        metadata={"origin": "vector_database"}
                    )
                except Exception as retrieval_error:
                    logger.warning(f"Vector database unavailable, falling back to LLM-only mode: {str(retrieval_error)}")
                    # Fallback to LLM without vector context
                    context_source = ContextSource(
                        type="llm_only",
                        content="",
                        retrieved_chunks_count=0,
                        metadata={"origin": "llm_fallback", "error": str(retrieval_error)}
                    )

            # Generate response using the context
            response = await self.generate_response(message, context_source)

            # Set confidence based on context source type
            if context_source.type == "llm_only":
                confidence = 0.5  # Lower confidence for LLM-only responses
            else:
                confidence = 0.8  # Higher confidence for context-aware responses

            return AgentResponse(
                response=response,
                source_chunks=[context_source.content] if context_source.content else [],
                confidence=confidence
            )

        except Exception as e:
            logger.error(f"Error processing message: {str(e)}")
            raise e

    async def generate_response(self, message: str, context_source: ContextSource) -> str:
        """
        Generate a response based on the message and context
        """
        try:
            # Create the system prompt based on context type
            if context_source.type == "selected_text":
                system_prompt = f"""
                You are an AI assistant that answers questions based strictly on the provided text.
                Answer the user's question based only on the following text:
                {context_source.content}

                Important rules:
                1. Answer only based on the provided text
                2. Do not use any external knowledge
                3. If the provided text doesn't contain the answer, say so
                4. Keep your answer concise and accurate
                """
            else:  # vector_retrieval
                system_prompt = f"""
                You are an AI assistant that answers questions based on the provided book content.
                Answer the user's question based on the following context from the book:
                {context_source.content}

                Important rules:
                1. Answer only based on the provided context
                2. Do not use any external knowledge
                3. If the provided context doesn't contain the answer, say so
                4. Keep your answer concise and accurate
                """

            # Create the messages for the LLM
            messages = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": message}
            ]

            # Call the Gemini API through the adapter
            response = self.gemini_client.chat_completions_create(
                model=settings.settings.gemini_model,
                messages=messages,
                max_tokens=500,
                temperature=0.3
            )

            # Extract the response content
            if response.choices and len(response.choices) > 0:
                generated_response = response.choices[0].message.content

                # Validate academic reliability of the response
                validation_result = validate_response_quality(generated_response, context_source.content)

                # Log validation results
                if not validation_result.get("is_reliable", True):
                    logger.warning(f"Response may not meet academic reliability standards: {validation_result.get('issues', [])}")

                return generated_response
            else:
                return "I couldn't generate a response based on the provided context."

        except Exception as e:
            logger.error(f"Error generating response: {str(e)}")
            raise e

    async def check_off_topic(self, message: str) -> bool:
        """
        Check if a message is off-topic (not related to book content)
        """
        try:
            # Create a prompt to determine if the question is off-topic
            system_prompt = """
            Determine if the user's question is related to book content or general knowledge.
            Answer with only 'off-topic' if the question is about general knowledge not related to books,
            otherwise answer with 'on-topic'.
            """

            messages = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": message}
            ]

            response = self.gemini_client.chat_completions_create(
                model=settings.settings.gemini_model,
                messages=messages,
                max_tokens=20,
                temperature=0.1
            )

            # Check if response has choices and the first choice has content
            if response.choices and len(response.choices) > 0 and response.choices[0].message.content:
                result = response.choices[0].message.content.strip().lower()
                # Check for both 'off-topic' and 'off' as the LLM might abbreviate
                return "off" in result and "on" not in result
            else:
                # If no content returned, assume it's on-topic to be safe
                return False

        except Exception as e:
            logger.error(f"Error checking if message is off-topic: {str(e)}")
            return False  # Default to on-topic if there's an error