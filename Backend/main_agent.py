import uvicorn
import os
import sys
from pathlib import Path

# Add the Agent directory to the path
agent_path = Path(__file__).parent / "Agent"
sys.path.insert(0, str(agent_path))

# Add the Retrieval/src directory to the path
retrieval_src_path = Path(__file__).parent / "Retrieval" / "src"
sys.path.insert(0, str(retrieval_src_path))

from Agent.main import app  # Import from Agent's main.py

if __name__ == "__main__":
    port = int(os.getenv("PORT", 8001))  # Use different port to avoid conflict
    host = os.getenv("HOST", "0.0.0.0")
    debug = os.getenv("DEBUG", "False").lower() == "true"

    uvicorn.run(
        app,
        host=host,
        port=port,
        reload=debug,
        log_level=os.getenv("LOG_LEVEL", "info").lower()
    )