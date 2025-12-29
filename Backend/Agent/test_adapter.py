"""
Test script to validate that the Agent SDK can successfully handshake with Gemini
"""
import asyncio
from .adapter import get_gemini_client
from .settings import settings


async def test_gemini_connection():
    """
    Test the connection to the Gemini API through LiteLLM
    """
    try:
        client = get_gemini_client()

        # Test a simple completion request
        messages = [
            {"role": "user", "content": "Hello, this is a test message to verify the connection."}
        ]

        response = client.chat_completions_create(
            model=settings.gemini_model,
            messages=messages,
            max_tokens=50,
            temperature=0.7
        )

        print("✅ Gemini connection test successful!")
        print(f"Response: {response.choices[0].message.content[:100]}...")
        return True

    except Exception as e:
        print(f"❌ Gemini connection test failed: {str(e)}")
        return False


if __name__ == "__main__":
    print("Testing Gemini connection...")
    success = asyncio.run(test_gemini_connection())

    if success:
        print("✅ Adapter handshake with Gemini successful!")
    else:
        print("❌ Adapter handshake with Gemini failed!")