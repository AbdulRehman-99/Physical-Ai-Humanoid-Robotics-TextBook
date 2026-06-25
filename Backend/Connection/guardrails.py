import logging
from typing import Optional, Any
from openai import AsyncOpenAI
from agents import input_guardrail, GuardrailFunctionOutput
from agents.run_context import RunContextWrapper

logger = logging.getLogger(__name__)


class GuardrailCtx:
    def __init__(
        self,
        book_content: str = "",
        client: Optional[AsyncOpenAI] = None,
        model_name: str = "",
    ):
        self.book_content = book_content
        self.client = client
        self.model_name = model_name


async def _check_off_topic(
    user_message: str,
    book_content: str,
    client: Optional[AsyncOpenAI],
    model_name: str,
) -> bool:
    if not user_message or not book_content or not client or not model_name:
        return False
    try:
        response = await client.chat.completions.create(
            model=model_name,
            messages=[
                {
                    "role": "system",
                    "content": (
                        "Answer only 'yes' or 'no': Can this question be answered "
                        "using the provided content?\n\n" + book_content[:1500]
                    ),
                },
                {"role": "user", "content": f"Question: {user_message}"},
            ],
            max_tokens=10,
            temperature=0.1,
        )
        answer = response.choices[0].message.content.strip().lower()
        return answer != "yes"
    except Exception as e:
        logger.warning(f"Off-topic guardrail check failed: {e}")
        return False


@input_guardrail
async def off_topic_guardrail(
    context: RunContextWrapper[GuardrailCtx],
    agent,
    input_data,
) -> GuardrailFunctionOutput:
    ctx = context.context
    user_msg = ""
    if isinstance(input_data, list):
        for item in reversed(input_data):
            if isinstance(item, dict) and item.get("role") == "user":
                user_msg = item.get("content", "")
                break
        if not user_msg:
            user_msg = str(input_data)
    else:
        user_msg = str(input_data)

    is_off_topic = await _check_off_topic(
        user_msg, ctx.book_content, ctx.client, ctx.model_name
    )
    return GuardrailFunctionOutput(
        output_info={"is_off_topic": is_off_topic},
        tripwire_triggered=is_off_topic,
    )
