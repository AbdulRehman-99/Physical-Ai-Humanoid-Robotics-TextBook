"""
Test script to validate that the Agent SDK can successfully handshake with OpenRouter
"""
import asyncio
from adapter import get_agent_adapter
import settings


async def test_openrouter_connection():
    """
    Test the connection to the OpenRouter API through the Agent SDK
    """
    try:
        adapter = get_agent_adapter()

        # Test raw chat completion
        messages = [
            {"role": "user", "content": "Hello, this is a test message to verify the connection."}
        ]

        response = await adapter.chat_completions_create(
            model=settings.settings.openrouter_model,
            messages=messages,
            max_tokens=50,
            temperature=0.7
        )

        print("✅ OpenRouter connection test successful!")
        print(f"Response: {response.choices[0].message.content[:100]}...")
        return True

    except Exception as e:
        print(f"❌ OpenRouter connection test failed: {str(e)}")
        return False


if __name__ == "__main__":
    print("Testing OpenRouter connection...")
    success = asyncio.run(test_openrouter_connection())
