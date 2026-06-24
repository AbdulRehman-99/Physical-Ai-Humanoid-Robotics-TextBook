import logging
import time
from typing import Optional, List, Dict, Any
from openai import AsyncOpenAI
from agents import Agent, Runner, set_default_openai_client, set_tracing_disabled
import settings


logger = logging.getLogger(__name__)


class OpenRouterAdapter:
    """
    Adapter that configures the OpenAI Agent SDK with OpenRouter as the provider.
    Provides both Agent SDK (Agent/Runner) and raw chat completions interfaces.
    """

    def __init__(self):
        api_key = settings.settings.openrouter_api_key
        base_url = settings.settings.openrouter_base_url

        # Disable tracing to avoid sending OpenRouter keys to OpenAI's trace endpoint
        set_tracing_disabled(True)

        self.client = AsyncOpenAI(
            base_url=base_url,
            api_key=api_key,
        )
        set_default_openai_client(self.client)

    def create_agent(
        self,
        instructions: str,
        model: Optional[str] = None,
    ) -> Agent:
        return Agent(
            name="RAG Agent",
            instructions=instructions,
            model=model or settings.settings.openrouter_model,
        )

    async def run_agent(self, agent: Agent, user_input: str) -> str:
        result = await Runner.run(agent, input=user_input)
        return result.final_output

    async def chat_completions_create(
        self,
        model: Optional[str] = None,
        messages: Optional[List[Dict[str, Any]]] = None,
        max_retries: int = 3,
        **kwargs,
    ):
        if model is None:
            model = settings.settings.openrouter_model

        last_exception = None
        for attempt in range(max_retries):
            try:
                response = await self.client.chat.completions.create(
                    model=model,
                    messages=messages,
                    **kwargs,
                )
                return response
            except Exception as e:
                last_exception = e
                logger.warning(
                    f"OpenRouter API call failed (attempt {attempt + 1}/{max_retries}): {str(e)}"
                )
                if attempt < max_retries - 1:
                    time.sleep(2 ** attempt)

        logger.error(f"All {max_retries} attempts to call OpenRouter API failed: {str(last_exception)}")
        raise last_exception


openrouter_adapter = OpenRouterAdapter()


def get_agent_adapter() -> OpenRouterAdapter:
    return openrouter_adapter
