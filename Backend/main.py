import uvicorn
import multiprocessing
import os
import sys
from pathlib import Path

def run_retrieval_test():
    """Run the Retrieval test script"""
    print("[Starting] Retrieval Test...")
    retrieval_path = Path(__file__).parent / "Retrieval"
    # We use a subprocess to avoid module name conflicts in the same interpreter
    import subprocess
    try:
        subprocess.run([sys.executable, "test_simple.py"], cwd=str(retrieval_path), check=True)
        print("[Completed] Retrieval Test finished.")
    except Exception as e:
        print(f"[Error] Retrieval Test failed: {e}")

def run_agent_api():
    """Run the Agent API on port 8001"""
    print("[Starting] Agent API on port 8001...")
    agent_path = Path(__file__).parent / "Agent"
    # Run as a separate process to avoid path/import issues
    import subprocess
    env = os.environ.copy()
    # Add Retrieval/src to PYTHONPATH so Agent can find it
    retrieval_src = str(Path(__file__).parent / "Retrieval" / "src")
    env["PYTHONPATH"] = f"{retrieval_src}{os.pathsep}{env.get('PYTHONPATH', '')}"
    
    subprocess.run([
        sys.executable, "-m", "uvicorn", "main:app", 
        "--host", "0.0.0.0", "--port", "8001"
    ], cwd=str(agent_path), env=env)

def run_connection_api():
    """Run the Connection API on port 8000 (Unified Entry)"""
    print("[Starting] Connection API (Main) on port 8000...")
    backend_root = Path(__file__).parent
    
    # Run directly in this process or via subprocess
    import subprocess
    env = os.environ.copy()
    env["PYTHONPATH"] = f"{str(backend_root)}{os.pathsep}{env.get('PYTHONPATH', '')}"
    
    subprocess.run([
        sys.executable, "-m", "uvicorn", "Connection.api:app", 
        "--host", "0.0.0.0", "--port", "8000"
    ], cwd=str(backend_root), env=env)

if __name__ == "__main__":
    # Use spawn to ensure clean environments for each process
    multiprocessing.set_start_method('spawn', force=True)
    
    p1 = multiprocessing.Process(target=run_retrieval_test)
    p2 = multiprocessing.Process(target=run_agent_api)
    p3 = multiprocessing.Process(target=run_connection_api)

    p1.start()
    p2.start()
    p3.start()

    try:
        p1.join()
        p2.join()
        p3.join()
    except KeyboardInterrupt:
        print("\n[Stopping] All services...")
        p1.terminate()
        p2.terminate()
        p3.terminate()
