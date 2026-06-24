import asyncio
import logging
from typing import Optional, List, Dict, AsyncGenerator
from openai import AsyncOpenAI
from agents import Agent, Runner, ModelSettings, set_default_openai_client, set_tracing_disabled
from agents.models.openai_chatcompletions import OpenAIChatCompletionsModel
from .models import ChatContext
from .config import AGENT_TIMEOUT, AGENT_RETRY_ATTEMPTS
import os

logger = logging.getLogger(__name__)


def _build_input_with_memory(
    message: str,
    memory: Optional[List[Dict[str, str]]] = None,
) -> list:
    items = []
    if memory:
        for turn in memory:
            items.append({"role": "user", "content": turn.get("user", "")})
            items.append({"role": "assistant", "content": turn.get("assistant", "")})
    items.append({"role": "user", "content": message})
    return items


class AgentWrapper:
    def __init__(self):
        self.api_key = os.getenv("OPENROUTER_API_KEY", "")
        self.base_url = os.getenv("OPENROUTER_BASE_URL", "https://openrouter.ai/api/v1")
        self.model_name = os.getenv("OPENROUTER_MODEL", "qwen-2.5-72b-instruct")

        if self.api_key:
            set_tracing_disabled(True)
            self.client = AsyncOpenAI(base_url=self.base_url, api_key=self.api_key)
            set_default_openai_client(self.client)
        else:
            logger.error("OPENROUTER_API_KEY not found in environment variables")
            self.client = None

    async def process_with_context(self, context: ChatContext) -> str:
        try:
            response = await self._call_agent_with_retry(context)
            return response
        except Exception as e:
            logger.error(f"Error processing with agent: {str(e)}")
            return "I can only answer questions about the book content. Please ask a question related to the book."

    async def process_with_context_streamed(
        self, context: ChatContext
    ) -> AsyncGenerator[str, None]:
        try:
            async for token in self._call_agent_streamed(context):
                yield token
        except Exception as e:
            logger.error(f"Error streaming with agent: {str(e)}")
            yield "I can only answer questions about the book content. Please ask a question related to the book."

    async def _call_agent_streamed(self, context: ChatContext) -> AsyncGenerator[str, None]:
        if not self.client:
            yield "I can only answer questions about the book content. Please ask a question related to the book."
            return

        instructions = self._build_instructions(context)
        model_obj = OpenAIChatCompletionsModel(
            model=self.model_name,
            openai_client=self.client,
        )
        agent = Agent(
            name="Book Assistant",
            instructions=instructions,
            model=model_obj,
            model_settings=ModelSettings(max_tokens=400),
        )

        input_items = _build_input_with_memory(context.user_message, context.memory)
        result = Runner.run_streamed(agent, input=input_items)

        async for event in result.stream_events():
            if event.type == "raw_response_event" and getattr(event.data, 'type', None) == "response.output_text.delta":
                if event.data.delta:
                    yield event.data.delta

        if result.final_output:
            yield result.final_output

    async def _call_agent_with_retry(self, context: ChatContext) -> str:
        last_error = None

        for attempt in range(AGENT_RETRY_ATTEMPTS):
            try:
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
                await asyncio.sleep(2 ** attempt)
            except Exception as e:
                logger.error(f"Agent call failed on attempt {attempt + 1}: {str(e)}")
                last_error = str(e)
                if attempt == AGENT_RETRY_ATTEMPTS - 1:
                    break
                await asyncio.sleep(2 ** attempt)

        if last_error == "timeout":
            return "The system is experiencing high load. Please try again later."
        else:
            return "I can only answer questions about the book content. Please ask a question related to the book."

    def _build_instructions(self, context: ChatContext) -> str:
        if context.selected_text:
            return f"""
            You are an AI assistant that must answer questions based ONLY on the provided book content.
            You have been given specific text from the book. Use ONLY this information to answer the question.
            Do not use any external knowledge or general information.

            Here is the book content you must use:
            {context.selected_text}

            Now answer this question using ONLY the above content:
            {context.user_message}

            If the provided content does not contain the answer, explicitly state that the information is not in the provided content.
            You MUST write a thorough answer of at most 800 words. You MUST use plain text headings, sub-headings, and bullet points. Do NOT use markdown formatting (no ##, no **, no backticks).
            """
        elif context.retrieved_chunks:
            chunk_texts = [chunk.content for chunk in context.retrieved_chunks]
            combined_context = " ".join(chunk_texts)

            return f"""
            You are an AI assistant that must answer questions based ONLY on the provided book content.
            You have been given specific text from the book. Use ONLY this information to answer the question.
            Do not use any external knowledge or general information.

            Here is the book content you must use:
            {combined_context[:6000]}

            Now answer this question using ONLY the above content:
            {context.user_message}

            If the provided content does not contain the answer, explicitly state that the information is not in the provided content.
            You MUST write a thorough answer of at most 800 words. You MUST use plain text headings, sub-headings, and bullet points. Do NOT use markdown formatting (no ##, no **, no backticks).
            """
        else:
            return f"""
            You are an AI assistant that can only answer questions about book content.
            Do not use any external knowledge.
            Question: {context.user_message}

            Since no specific content was provided, explicitly state that you couldn't find relevant content in the book.
            You MUST write a thorough answer of at most 800 words. You MUST use plain text headings, sub-headings, and bullet points. Do NOT use markdown formatting (no ##, no **, no backticks).
            """

    async def _call_agent_internal(self, context: ChatContext) -> str:
        if not self.client:
            return "I can only answer questions about the book content. Please ask a question related to the book."

        try:
            # Off-topic detection via keyword check
            off_topic_indicators = ["weather", "joke", "unrelated", "random", "movie", "sports", "food"]
            user_message_lower = context.user_message.lower()

            if any(indicator in user_message_lower for indicator in off_topic_indicators):
                return "I can only answer questions about the book content. Please ask a question related to the book."

            instructions = self._build_instructions(context)

            model_obj = OpenAIChatCompletionsModel(
                model=self.model_name,
                openai_client=self.client,
            )
            agent = Agent(
                name="Book Assistant",
                instructions=instructions,
                model=model_obj,
                model_settings=ModelSettings(max_tokens=400),
            )

            input_items = _build_input_with_memory(context.user_message, context.memory)
            result = await Runner.run(agent, input=input_items)
            generated_text = result.final_output.strip()

            logger.info(f"OpenRouter returned: {generated_text[:200]}...")
            return generated_text

        except Exception as e:
            logger.error(f"Error calling OpenRouter via Agent SDK: {str(e)}")
            return "I can only answer questions about the book content. Please ask a question related to the book."
