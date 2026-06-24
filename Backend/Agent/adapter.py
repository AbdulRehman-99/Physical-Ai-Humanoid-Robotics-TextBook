import logging
import time
from typing import Optional, List, Dict, Any, AsyncGenerator
from openai import AsyncOpenAI
from agents import (
    Agent, Runner, ModelSettings, set_default_openai_client, set_tracing_disabled,
    input_guardrail, output_guardrail, GuardrailFunctionOutput,
    InputGuardrail, OutputGuardrail,
)
from agents.models.openai_chatcompletions import OpenAIChatCompletionsModel
import settings


logger = logging.getLogger(__name__)


def _build_input_with_memory(
    message: str,
    memory: Optional[List[Dict[str, str]]] = None,
) -> List[Dict[str, str]]:
    """
    Build a list of input items with conversation history.
    memory is a list of {role, content} dicts (alternating user/assistant).
    """
    items: List[Dict[str, str]] = []
    if memory:
        for turn in memory:
            items.append({"role": "user", "content": turn.get("user", "")})
            items.append({"role": "assistant", "content": turn.get("assistant", "")})
    items.append({"role": "user", "content": message})
    return items


def _trim_memory(
    memory: Optional[List[Dict[str, str]]],
    max_turns: int = 5,
) -> Optional[List[Dict[str, str]]]:
    """Keep only the last max_turns exchanges."""
    if not memory:
        return None
    return memory[-max_turns:]


class OpenRouterAdapter:
    """
    Adapter that configures the OpenAI Agent SDK with OpenRouter as the provider.
    """

    def __init__(self):
        api_key = settings.settings.openrouter_api_key
        base_url = settings.settings.openrouter_base_url

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
        max_tokens: int = 400,
        input_guardrails: Optional[List[InputGuardrail]] = None,
        output_guardrails: Optional[List[OutputGuardrail]] = None,
    ) -> Agent:
        model_name = model or settings.settings.openrouter_model
        model_obj = OpenAIChatCompletionsModel(
            model=model_name,
            openai_client=self.client,
        )
        return Agent(
            name="RAG Agent",
            instructions=instructions,
            model=model_obj,
            model_settings=ModelSettings(max_tokens=max_tokens),
            input_guardrails=input_guardrails,
            output_guardrails=output_guardrails,
        )

    async def run_agent(
        self,
        agent: Agent,
        user_input: str,
        memory: Optional[List[Dict[str, str]]] = None,
    ) -> str:
        trimmed_memory = _trim_memory(memory)
        input_items = _build_input_with_memory(user_input, trimmed_memory)
        result = await Runner.run(agent, input=input_items)
        return result.final_output

    async def run_agent_streamed(
        self,
        agent: Agent,
        user_input: str,
        memory: Optional[List[Dict[str, str]]] = None,
    ) -> AsyncGenerator[str, None]:
        """
        Run the agent with streaming support.
        Yields content tokens as they arrive.
        """
        trimmed_memory = _trim_memory(memory)
        input_items = _build_input_with_memory(user_input, trimmed_memory)
        result = Runner.run_streamed(agent, input=input_items)

        async for event in result.stream_events():
            if event.type == "raw_response_event" and getattr(event.data, 'type', None) == "response.output_text.delta":
                if event.data.delta:
                    yield event.data.delta

        # Yield the final output if nothing was streamed (edge case)
        if result.final_output:
            yield result.final_output

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
