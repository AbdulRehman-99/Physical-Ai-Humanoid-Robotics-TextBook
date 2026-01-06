from fastapi import FastAPI, HTTPException, Depends, Request
from fastapi.middleware.cors import CORSMiddleware
from typing import Dict, Any
import asyncio
import os
from dotenv import load_dotenv

# Load environment variables from .env file in the project root
load_dotenv(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), '.env'))

from .models import ChatRequest, ChatResponse
from .agent_wrapper import AgentWrapper
from .context_switcher import ContextSwitcher
from .response_formatter import ResponseFormatter
from .config import TIMEOUT_SECONDS, RETRY_ATTEMPTS, RATE_LIMIT_MAX_REQUESTS, RATE_LIMIT_WINDOW
import logging
from collections import defaultdict
import time

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Simple in-memory rate limiter
class RateLimiter:
    def __init__(self, max_requests: int, window: int):
        self.max_requests = max_requests
        self.window = window
        self.requests = defaultdict(list)

    def is_allowed(self, identifier: str) -> bool:
        now = time.time()
        # Remove old requests outside the window
        self.requests[identifier] = [req_time for req_time in self.requests[identifier] if now - req_time < self.window]

        if len(self.requests[identifier]) < self.max_requests:
            self.requests[identifier].append(now)
            return True
        return False

rate_limiter = RateLimiter(RATE_LIMIT_MAX_REQUESTS, RATE_LIMIT_WINDOW)

app = FastAPI(title="Chat API", description="API for connecting Docusaurus frontend with FastAPI RAG backend using ChatKit as frontend-only layer")

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize components
agent_wrapper = AgentWrapper()
context_switcher = ContextSwitcher()
response_formatter = ResponseFormatter()

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(chat_request: ChatRequest, request: Request) -> ChatResponse:
    """
    Process user chat messages and return AI-generated, book-grounded responses
    """
    try:
        # Rate limiting based on IP address
        client_ip = request.client.host
        if not rate_limiter.is_allowed(client_ip):
            logger.warning(f"Rate limit exceeded for IP: {client_ip}")
            return ChatResponse(
                response="Rate limit exceeded. Please try again later.",
                sources=[]
            )

        # Input sanitization for security
        sanitized_message = chat_request.message.strip()
        sanitized_selected_text = chat_request.selected_text.strip() if chat_request.selected_text else None

        # Validate the request
        if not sanitized_message or sanitized_message == "":
            # Handle empty message
            return ChatResponse(
                response="Please provide a question or message to get started.",
                sources=[]
            )

        # Validate for extremely long messages
        if len(sanitized_message) > 10000:  # 10k character limit
            return ChatResponse(
                response="Message is too long. Please keep your message under 10,000 characters.",
                sources=[]
            )

        # Basic security check: prevent potential injection attacks
        if any(injection_pattern in sanitized_message.lower() for injection_pattern in ["<script", "javascript:", "onerror", "alert("]):
            logger.warning(f"Potential injection attempt from IP: {client_ip}")
            return ChatResponse(
                response="Invalid input detected. Please ask a question related to the book.",
                sources=[]
            )

        # Determine context using the context switcher
        chat_context = await context_switcher.determine_context_async(
            user_message=sanitized_message,
            selected_text=sanitized_selected_text
        )

        # Process with agent
        agent_response = await agent_wrapper.process_with_context(chat_context)

        # Format the response
        formatted_response = response_formatter.format_response(
            agent_response=agent_response,
            context=chat_context
        )

        return formatted_response

    except Exception as e:
        logger.error(f"Error processing chat request: {str(e)}")
        # Return polite refusal without HTTP error
        return ChatResponse(
            response="I can only answer questions about the book content. Please ask a question related to the book.",
            sources=[]
        )


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "healthy", "timestamp": time.time()}


# Error handling middleware
@app.exception_handler(404)
async def custom_http_exception_handler(request: Request, exc: HTTPException):
    return ChatResponse(
        response="I can only answer questions about the book content. Please ask a question related to the book.",
        sources=[]
    )