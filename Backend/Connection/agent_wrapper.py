import asyncio
import logging
from openai import AsyncOpenAI
from agents import Agent, Runner, set_default_openai_client, set_tracing_disabled
from .models import ChatContext
from .config import AGENT_TIMEOUT, AGENT_RETRY_ATTEMPTS
import os

logger = logging.getLogger(__name__)

class AgentWrapper:
    def __init__(self):
        self.api_key = os.getenv("OPENROUTER_API_KEY", "")
        self.base_url = os.getenv("OPENROUTER_BASE_URL", "https://openrouter.ai/api/v1")
        self.model_name = os.getenv("OPENROUTER_MODEL", "qwen-2.5-72b-instruct")

        if self.api_key:
            # Disable tracing to avoid sending OpenRouter keys to OpenAI's trace endpoint
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

    async def _call_agent_internal(self, context: ChatContext) -> str:
        if not self.client:
            return "I can only answer questions about the book content. Please ask a question related to the book."

        try:
            # Off-topic detection via keyword check
            off_topic_indicators = ["weather", "joke", "unrelated", "random", "movie", "sports", "food"]
            user_message_lower = context.user_message.lower()

            if any(indicator in user_message_lower for indicator in off_topic_indicators):
                return "I can only answer questions about the book content. Please ask a question related to the book."

            # Build instructions based on context
            if context.selected_text:
                instructions = f"""
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
                chunk_texts = [chunk.content for chunk in context.retrieved_chunks]
                combined_context = " ".join(chunk_texts)

                instructions = f"""
                You are an AI assistant that must answer questions based ONLY on the provided book content.
                You have been given specific text from the book. Use ONLY this information to answer the question.
                Do not use any external knowledge or general information.

                Here is the book content you must use:
                {combined_context[:2000]}

                Now answer this question using ONLY the above content:
                {context.user_message}

                If the provided content does not contain the answer, explicitly state that the information is not in the provided content.
                """
            else:
                instructions = f"""
                You are an AI assistant that can only answer questions about book content.
                Do not use any external knowledge.
                Question: {context.user_message}

                Since no specific content was provided, explicitly state that you couldn't find relevant content in the book.
                """

            # Use OpenAI Agent SDK with max_turns=1 for simple Q&A
            agent = Agent(
                name="Book Assistant",
                instructions=instructions,
                model=self.model_name,
            )

            result = await Runner.run(agent, input=context.user_message)
            generated_text = result.final_output.strip()

            logger.info(f"OpenRouter returned: {generated_text[:200]}...")
            return generated_text

        except Exception as e:
            logger.error(f"Error calling OpenRouter via Agent SDK: {str(e)}")
            return "I can only answer questions about the book content. Please ask a question related to the book."
