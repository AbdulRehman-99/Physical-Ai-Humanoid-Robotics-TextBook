import uvicorn
import os
import sys
from pathlib import Path

# Hugging Face Spaces expect a single main entry point
# We configure this to run ONLY the Connection API on port 7860
port = int(os.getenv("PORT", 7860))
host = "0.0.0.0"

print(f"--- Starting Hugging Face Production Backend on {host}:{port} ---")

# Ensure the backend directory is in the path
backend_root = Path(__file__).parent
sys.path.insert(0, str(backend_root))

# Run the Connection API
if __name__ == "__main__":
    uvicorn.run("Connection.api:app", host=host, port=port, log_level="info")
