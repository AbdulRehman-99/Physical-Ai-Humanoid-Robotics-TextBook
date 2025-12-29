import logging
from typing import Callable, Any, Optional
from functools import wraps
from settings import settings


logger = logging.getLogger(__name__)


class QdrantError(Exception):
    """Custom exception for Qdrant-related errors"""
    pass


class CircuitBreaker:
    """
    Circuit breaker pattern implementation for Qdrant calls
    """
    def __init__(self, failure_threshold: int = 5, recovery_timeout: int = 60):
        self.failure_threshold = failure_threshold
        self.recovery_timeout = recovery_timeout
        self.failure_count = 0
        self.last_failure_time = None
        self.state = "CLOSED"  # CLOSED, OPEN, HALF_OPEN

    def call(self, func: Callable, *args, **kwargs) -> Any:
        if self.state == "OPEN":
            import time
            if time.time() - self.last_failure_time > self.recovery_timeout:
                self.state = "HALF_OPEN"
            else:
                raise QdrantError("Circuit breaker is OPEN - service temporarily unavailable")

        try:
            result = func(*args, **kwargs)
            if self.state == "HALF_OPEN":
                self.state = "CLOSED"
                self.failure_count = 0
            return result
        except Exception as e:
            self.failure_count += 1
            self.last_failure_time = __import__('time').time()

            if self.failure_count >= self.failure_threshold:
                self.state = "OPEN"
                logger.error(f"Circuit breaker OPEN after {self.failure_count} failures")

            raise e


# Global circuit breaker for Qdrant
qdrant_circuit_breaker = CircuitBreaker()


def handle_qdrant_errors(func):
    """
    Decorator to handle Qdrant errors with comprehensive error handling
    """
    @wraps(func)
    def wrapper(*args, **kwargs):
        try:
            return qdrant_circuit_breaker.call(func, *args, **kwargs)
        except Exception as e:
            error_msg = f"Qdrant operation failed: {str(e)}"
            logger.error(error_msg)

            # Log the error for monitoring
            logger.error(f"Qdrant error details: function={func.__name__}, args={args}, kwargs={kwargs}")

            # Raise a custom exception that can be handled by upstream code
            raise QdrantError(error_msg) from e

    return wrapper


def add_comprehensive_error_handling():
    """
    Apply comprehensive error handling to Qdrant operations
    """
    # This function would typically be used to wrap Qdrant client methods
    # In practice, we'd apply the @handle_qdrant_errors decorator to specific functions
    pass


def log_error(error: Exception, context: str = ""):
    """
    Log errors with appropriate context
    """
    logger.error(f"Error in {context}: {str(error)}", exc_info=True)


def handle_fallback_logic(error: Exception, fallback_func: Optional[Callable] = None):
    """
    Handle fallback logic when primary operation fails
    """
    logger.warning(f"Primary operation failed: {str(error)}, attempting fallback")

    if fallback_func:
        try:
            result = fallback_func()
            logger.info("Fallback operation successful")
            return result
        except Exception as fallback_error:
            logger.error(f"Fallback operation also failed: {str(fallback_error)}")
            raise fallback_error
    else:
        # Default fallback behavior
        logger.info("No fallback function provided, returning None")
        return None