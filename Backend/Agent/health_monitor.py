import asyncio
import logging
from typing import Dict, Any
import sys
import os
from pathlib import Path

# Add the Retrieval/src directory to the path
retrieval_src_path = Path(__file__).parent.parent / "Retrieval" / "src"
sys.path.insert(0, str(retrieval_src_path))

from retrieval import get_retrieval_service
from settings import settings


logger = logging.getLogger(__name__)


class HealthMonitor:
    """
    Health monitoring for vector database connectivity
    """

    def __init__(self):
        self.retrieval_service = get_retrieval_service()
        self.is_healthy = True
        self.last_check_time = None

    async def check_qdrant_health(self) -> Dict[str, Any]:
        """
        Check the health of the Qdrant vector database connection
        """
        try:
            # Perform a simple operation to test connectivity
            # We'll try to get a small search result to verify the connection
            test_result = self.retrieval_service.search_similar("test", top_k=1)

            # If we get here, the connection is working
            self.is_healthy = True
            self.last_check_time = __import__('time').time()

            return {
                "status": "healthy",
                "connected": True,
                "message": "Qdrant connection is healthy",
                "last_check": self.last_check_time
            }
        except Exception as e:
            logger.error(f"Qdrant health check failed: {str(e)}")
            self.is_healthy = False
            self.last_check_time = __import__('time').time()

            return {
                "status": "unhealthy",
                "connected": False,
                "message": f"Qdrant connection error: {str(e)}",
                "last_check": self.last_check_time
            }

    async def periodic_health_check(self, interval: int = 30):
        """
        Perform periodic health checks on the vector database
        """
        while True:
            try:
                health_status = await self.check_qdrant_health()
                if not health_status["connected"]:
                    logger.warning("Vector database health check failed")
                else:
                    logger.info("Vector database health check passed")

                # Wait for the specified interval before next check
                await asyncio.sleep(interval)
            except Exception as e:
                logger.error(f"Error during periodic health check: {str(e)}")
                # Continue the loop even if there's an error in the check itself
                await asyncio.sleep(interval)

    def get_health_status(self) -> Dict[str, Any]:
        """
        Get the current health status
        """
        return {
            "is_healthy": self.is_healthy,
            "last_check_time": self.last_check_time,
            "service": "qdrant_vector_database"
        }


# Singleton instance
health_monitor = HealthMonitor()


def get_health_monitor():
    """
    Get the health monitor instance
    """
    return health_monitor


# Function to start the periodic health checks if needed
async def start_health_monitoring():
    """
    Start the periodic health monitoring in the background
    """
    monitor = get_health_monitor()
    await monitor.periodic_health_check()