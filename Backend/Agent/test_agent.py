import asyncio
import sys
from pathlib import Path

# Add necessary paths
agent_path = Path(__file__).parent
sys.path.insert(0, str(agent_path))

# Add the Retrieval/src directory to the path
retrieval_src_path = agent_path / "Retrieval" / "src"
sys.path.insert(0, str(retrieval_src_path))

# Import modules directly to avoid relative import issues
import adapter
import settings
from retrieval import get_retrieval_service

# Import the agent module by executing it as a module
import importlib.util
agent_spec = importlib.util.spec_from_file_location("agent", agent_path / "agent.py")
agent_module = importlib.util.module_from_spec(agent_spec)
agent_spec.loader.exec_module(agent_module)

async def test_agent():
    try:
        print("Initializing RAG Agent...")
        rag_agent = agent_module.RAGAgent()
        print("RAG Agent initialized successfully")

        print("Testing with a sample question...")
        response = await rag_agent.process_message("What is ROS 2?")
        print(f"Response: {response.response}")
        print(f"Source chunks: {response.source_chunks}")
        print(f"Confidence: {response.confidence}")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

# Run the test
if __name__ == "__main__":
    asyncio.run(test_agent())