from typing import Optional, List, Dict, Any, AsyncGenerator
from pydantic import BaseModel, Field
from adapter import get_agent_adapter
import sys
import os
import importlib.util
from pathlib import Path

# Add the Retrieval/src directory to the path
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
    response: str
    source_chunks: List[str]
    confidence: float


class ContextSource(BaseModel):
    type: str
    content: str
    retrieved_chunks_count: int = 0
    metadata: Dict[str, Any] = {}


class RAGAgent:
    """
    RAG Agent with streaming, memory (last 5 turns), and guardrails.
    """

    def __init__(self):
        self.adapter = get_agent_adapter()
        self.retrieval_service = get_retrieval_service()

    def _build_instructions(self, context_source: ContextSource) -> str:
        """Build instructions with length constraint."""
        if context_source.type == "selected_text":
            return f"""
            You are an AI assistant that answers questions based strictly on the provided text.
            Answer the user's question based only on the following text:
            {context_source.content}

            Important rules:
            1. Answer only based on the provided text
            2. Do not use any external knowledge
            3. If the provided text doesn't contain the answer, say so
            4. Use plain text only. Format each section like this example:

            Based on the provided textbook content, Humanoid Robotics is:
            - A multidisciplinary field combining mechanical engineering, AI, and cognitive science
            - Focused on creating machines that mimic human form, movement, and behavior
            - Designed to operate effectively in human environments

            Key concepts mentioned:
            - Embodied Intelligence: Intelligence arises from agent-environment interaction
            - Bipedal Locomotion: Dynamic balance control for two-legged walking
            - Human-Robot Interaction: Natural interfaces for intuitive communication

            Purpose:
            Humanoid robots serve as research platforms and practical assistants across healthcare, education, and disaster response, bridging the gap between digital AI and physical action.

            Keep the answer very short (80-100 words, max 2 sections).
            Do NOT use markdown symbols like #, ##, **, or backticks.
            """
        else:
            return f"""
            You are an AI assistant that answers questions based on the provided book content.
            Answer the user's question based on the following context from the book:
            {context_source.content}

            Important rules:
            1. Answer only based on the provided context
            2. Do not use any external knowledge
            3. If the provided context doesn't contain the answer, say so
            4. Use plain text only. Format each section like this example:

            Based on the provided textbook content, Humanoid Robotics is:
            - A multidisciplinary field combining mechanical engineering, AI, and cognitive science
            - Focused on creating machines that mimic human form, movement, and behavior
            - Designed to operate effectively in human environments

            Key concepts mentioned:
            - Embodied Intelligence: Intelligence arises from agent-environment interaction
            - Bipedal Locomotion: Dynamic balance control for two-legged walking
            - Human-Robot Interaction: Natural interfaces for intuitive communication

            Purpose:
            Humanoid robots serve as research platforms and practical assistants across healthcare, education, and disaster response, bridging the gap between digital AI and physical action.

            Keep the answer very short (80-100 words, max 2 sections).
            Do NOT use markdown symbols like #, ##, **, or backticks.
            """

    async def process_message(
        self,
        message: str,
        selected_text: Optional[str] = None,
        memory: Optional[List[Dict[str, str]]] = None,
    ) -> AgentResponse:
        """Process a message with optional context and memory."""
        try:
            if not selected_text or selected_text.strip() == "":
                is_off_topic = await self.check_off_topic(message)
                if is_off_topic:
                    return AgentResponse(
                        response="Ask the question related to the book",
                        source_chunks=[],
                        confidence=0.5,
                    )

            if selected_text is not None and selected_text.strip() != "":
                context_source = ContextSource(
                    type="selected_text",
                    content=selected_text,
                    retrieved_chunks_count=0,
                    metadata={"origin": "user_selection"},
                )
            else:
                try:
                    retrieved_context = self.retrieval_service.get_book_context(
                        message, top_k=settings.settings.top_k
                    )
                    context_source = ContextSource(
                        type="vector_retrieval",
                        content=retrieved_context,
                        retrieved_chunks_count=(
                            len(retrieved_context.split()) if retrieved_context else 0
                        ),
                        metadata={"origin": "vector_database"},
                    )
                except Exception as retrieval_error:
                    logger.warning(
                        f"Vector database unavailable, falling back to LLM-only mode: {str(retrieval_error)}"
                    )
                    context_source = ContextSource(
                        type="llm_only",
                        content="",
                        retrieved_chunks_count=0,
                        metadata={"origin": "llm_fallback", "error": str(retrieval_error)},
                    )

            response = await self.generate_response(message, context_source, memory)

            if context_source.type == "llm_only":
                confidence = 0.5
            else:
                confidence = 0.8

            return AgentResponse(
                response=response,
                source_chunks=[context_source.content] if context_source.content else [],
                confidence=confidence,
            )

        except Exception as e:
            logger.error(f"Error processing message: {str(e)}")
            raise e

    async def process_message_streamed(
        self,
        message: str,
        selected_text: Optional[str] = None,
        memory: Optional[List[Dict[str, str]]] = None,
    ) -> AsyncGenerator[str, None]:
        """Stream response tokens for a message with optional context and memory."""
        try:
            if not selected_text or selected_text.strip() == "":
                is_off_topic = await self.check_off_topic(message)
                if is_off_topic:
                    yield "Ask the question related to the book"
                    return

            if selected_text is not None and selected_text.strip() != "":
                context_source = ContextSource(
                    type="selected_text",
                    content=selected_text,
                    metadata={"origin": "user_selection"},
                )
            else:
                try:
                    retrieved_context = self.retrieval_service.get_book_context(
                        message, top_k=settings.settings.top_k
                    )
                    context_source = ContextSource(
                        type="vector_retrieval",
                        content=retrieved_context,
                        metadata={"origin": "vector_database"},
                    )
                except Exception as retrieval_error:
                    logger.warning(
                        f"Vector database unavailable, falling back to LLM-only mode: {str(retrieval_error)}"
                    )
                    context_source = ContextSource(
                        type="llm_only",
                        content="",
                        metadata={"origin": "llm_fallback"},
                    )

            instructions = self._build_instructions(context_source)

            agent = self.adapter.create_agent(
                instructions=instructions,
                model=settings.settings.openrouter_model,
                max_tokens=150,
            )

            async for token in self.adapter.run_agent_streamed(agent, message, memory):
                yield token

        except Exception as e:
            logger.error(f"Error in streamed processing: {str(e)}")
            yield "I encountered an error processing your request."

    async def generate_response(
        self,
        message: str,
        context_source: ContextSource,
        memory: Optional[List[Dict[str, str]]] = None,
    ) -> str:
        """Generate a response using the Agent SDK with memory."""
        try:
            instructions = self._build_instructions(context_source)

            agent = self.adapter.create_agent(
                instructions=instructions,
                model=settings.settings.openrouter_model,
                max_tokens=150,
            )
            generated_response = await self.adapter.run_agent(
                agent, message, memory
            )

            validation_result = validate_response_quality(
                generated_response, context_source.content
            )

            if not validation_result.get("is_reliable", True):
                logger.warning(
                    "Response may not meet academic reliability standards: "
                    f"{validation_result.get('issues', [])}"
                )

            return generated_response

        except Exception as e:
            logger.error(f"Error generating response: {str(e)}")
            raise e

    async def check_off_topic(self, message: str) -> bool:
        """Check if a message is off-topic."""
        try:
            system_prompt = """
            Determine if the user's question is related to book content or general knowledge.
            Answer with only 'off-topic' if the question is about general knowledge not related to books,
            otherwise answer with 'on-topic'.
            """

            messages = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": message},
            ]

            response = await self.adapter.chat_completions_create(
                model=settings.settings.openrouter_model,
                messages=messages,
                max_tokens=20,
                temperature=0.1,
            )

            if (
                response.choices
                and len(response.choices) > 0
                and response.choices[0].message.content
            ):
                result = response.choices[0].message.content.strip().lower()
                return "off" in result and "on" not in result
            else:
                return False

        except Exception as e:
            logger.error(f"Error checking if message is off-topic: {str(e)}")
            return False
