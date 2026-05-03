import uvicorn
import os
import sys
from pathlib import Path

# Hugging Face Spaces MUST listen on port 7860 for the health check to pass.
# Even if a PORT environment variable exists, we force 7860 here to ensure
# production stability on the HF platform.
port = 7860
host = "0.0.0.0"

print(f"--- Starting Hugging Face Production Backend on {host}:{port} ---")

# Ensure the backend directory is in the path for module resolution
backend_root = Path(__file__).parent
sys.path.insert(0, str(backend_root))

# Run the Connection API
if __name__ == "__main__":
    uvicorn.run("Connection.api:app", host=host, port=port, log_level="info")
