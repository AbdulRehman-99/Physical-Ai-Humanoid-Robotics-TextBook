import time
from typing import Dict
from fastapi import Request, HTTPException
from collections import defaultdict, deque
import settings
import logging


logger = logging.getLogger(__name__)


class RateLimiter:
    """
    Rate limiting middleware to support 100 concurrent users
    """
    def __init__(self, max_requests: int = 100, window_size: int = 60):
        self.max_requests = max_requests  # Max requests per window
        self.window_size = window_size  # Time window in seconds
        self.requests: Dict[str, deque] = defaultdict(deque)  # Store request times by IP

    def is_allowed(self, identifier: str) -> bool:
        """
        Check if a request from the given identifier is allowed
        """
        current_time = time.time()

        # Remove requests older than the window size
        while (self.requests[identifier] and
               current_time - self.requests[identifier][0] > self.window_size):
            self.requests[identifier].popleft()

        # Check if the number of requests is within the limit
        if len(self.requests[identifier]) < self.max_requests:
            self.requests[identifier].append(current_time)
            return True

        return False


# Initialize the rate limiter
rate_limiter = RateLimiter(max_requests=100, window_size=60)


def rate_limit_middleware(request: Request) -> None:
    """
    Rate limiting middleware function
    """
    # Get client IP address
    client_ip = request.client.host

    # Check if the request is allowed
    if not rate_limiter.is_allowed(client_ip):
        logger.warning(f"Rate limit exceeded for IP: {client_ip}")
        raise HTTPException(
            status_code=429,
            detail="Rate limit exceeded. Please try again later."
        )


# For integration with FastAPI
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import Response
from starlette.requests import Request as StarletteRequest


class RateLimitMiddleware(BaseHTTPMiddleware):
    async def dispatch(self, request: StarletteRequest, call_next):
        client_ip = request.client.host

        if not rate_limiter.is_allowed(client_ip):
            logger.warning(f"Rate limit exceeded for IP: {client_ip}")
            return Response(
                content="Rate limit exceeded. Please try again later.",
                status_code=429
            )

        response = await call_next(request)
        return response