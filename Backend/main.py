import uvicorn
import multiprocessing
import os
import sys
import subprocess
import time
from pathlib import Path

def run_retrieval_test():
    """Run the Retrieval test script"""
    print("[Starting] Retrieval Test...")
    retrieval_path = Path(__file__).parent / "Retrieval"
    os.chdir(retrieval_path)
    # Ensure sys.path includes the current dir for imports
    sys.path.insert(0, str(retrieval_path))
    try:
        # Import and run the test
        import test_simple
        print("[Completed] Retrieval Test finished.")
    except Exception as e:
        print(f"[Error] Retrieval Test failed: {e}")

def run_agent_api():
    """Run the Agent API on port 8001"""
    print("[Starting] Agent API on port 8001...")
    agent_path = Path(__file__).parent / "Agent"
    # Move to the directory so uvicorn can find 'main:app'
    os.chdir(agent_path)
    # Add Retrieval/src to sys.path as expected by Agent/main.py
    retrieval_src = Path(__file__).parent / "Retrieval" / "src"
    sys.path.insert(0, str(retrieval_src))
    
    uvicorn.run("main:app", host="0.0.0.0", port=8001, log_level="info")

def run_connection_api():
    """Run the Connection API on the port provided by environment (default 7860 for HF)"""
    port = int(os.getenv("PORT", 8002))
    print(f"[Starting] Connection API on port {port}...")
    backend_root = Path(__file__).parent
    os.chdir(backend_root)
    # Ensure sys.path is correct
    sys.path.insert(0, str(backend_root))
    # Connection/api.py adds its own paths, so we just run it
    uvicorn.run("Connection.api:app", host="0.0.0.0", port=port, log_level="info")

if __name__ == "__main__":
    # Create processes
    p1 = multiprocessing.Process(target=run_retrieval_test)
    p2 = multiprocessing.Process(target=run_agent_api)
    p3 = multiprocessing.Process(target=run_connection_api)

    # Start processes
    p1.start()
    p2.start()
    p3.start()

    # Keep the main process alive until interrupted
    try:
        p1.join()
        p2.join()
        p3.join()
    except KeyboardInterrupt:
        print("\n[Stopping] Backend services...")
        p1.terminate()
        p2.terminate()
        p3.terminate()
        print("[Stopped] All services.")
