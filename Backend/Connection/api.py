from fastapi import FastAPI, HTTPException, Depends, Request
from fastapi.responses import StreamingResponse
from fastapi.middleware.cors import CORSMiddleware
from typing import Dict, Any, Optional
import asyncio
import os
import json
import uuid
import time
from datetime import datetime, timedelta
from dotenv import load_dotenv

# Load environment variables from .env file in the project root
load_dotenv(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), '.env'))

from .models import ChatRequest, ChatResponse, ChatContext
from .agent_wrapper import AgentWrapper, OFF_TOPIC_SENTINEL
from .context_switcher import ContextSwitcher
from .response_formatter import ResponseFormatter
from .config import (
    TIMEOUT_SECONDS, RETRY_ATTEMPTS,
    RATE_LIMIT_MAX_REQUESTS, RATE_LIMIT_WINDOW,
    SESSION_MEMORY_TURNS, SESSION_TTL_SECONDS,
)
import logging
from collections import defaultdict

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# In-memory conversation sessions
conversation_sessions: Dict[str, list] = {}
session_last_access: Dict[str, float] = {}


def _get_or_create_session(session_id: Optional[str]) -> str:
    if session_id and session_id in conversation_sessions:
        session_last_access[session_id] = time.time()
        return session_id
    new_id = session_id or str(uuid.uuid4())
    if new_id not in conversation_sessions:
        conversation_sessions[new_id] = []
        session_last_access[new_id] = time.time()
    return new_id


def _update_memory(session_id: str, user_msg: str, assistant_msg: str):
    conversation_sessions[session_id].append({"user": user_msg, "assistant": assistant_msg})
    session_last_access[session_id] = time.time()
    if len(conversation_sessions[session_id]) > SESSION_MEMORY_TURNS:
        conversation_sessions[session_id] = conversation_sessions[session_id][-SESSION_MEMORY_TURNS:]


async def _session_cleanup_task():
    """Periodically evict sessions idle longer than SESSION_TTL_SECONDS."""
    while True:
        await asyncio.sleep(300)  # run every 5 minutes
        now = time.time()
        stale = [
            sid for sid, last in session_last_access.items()
            if now - last > SESSION_TTL_SECONDS
        ]
        for sid in stale:
            conversation_sessions.pop(sid, None)
            session_last_access.pop(sid, None)
        if stale:
            logger.info(f"Evicted {len(stale)} stale sessions")


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


@app.on_event("startup")
async def startup():
    asyncio.create_task(_session_cleanup_task())
    logger.info("Session cleanup task started")

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

        session_id = _get_or_create_session(chat_request.session_id)

        # Determine context using the context switcher
        chat_context = await context_switcher.determine_context_async(
            user_message=sanitized_message,
            selected_text=sanitized_selected_text
        )
        chat_context.memory = conversation_sessions.get(session_id)

        # Process with agent
        agent_response = await agent_wrapper.process_with_context(chat_context)

        # Check if guardrail triggered (off-topic query)
        if agent_response == OFF_TOPIC_SENTINEL:
            return ChatResponse(
                response="I can only answer questions about the book content. Please ask a question related to the book.",
                sources=[],
                is_off_topic=True,
            )

        # Update memory
        _update_memory(session_id, sanitized_message, agent_response)

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


@app.post("/chat/stream")
async def chat_stream_endpoint(chat_request: ChatRequest, request: Request):
    """
    Stream response tokens via SSE with optional session memory for conversation continuity.
    Uses a simple session_id header or IP-based session.
    """
    try:
        session_id = _get_or_create_session(chat_request.session_id)
        sanitized_message = chat_request.message.strip()
        sanitized_selected_text = chat_request.selected_text.strip() if chat_request.selected_text else None

        if not sanitized_message:
            async def _empty():
                yield json.dumps({"error": "Empty message"}) + "\n"
            return StreamingResponse(_empty(), media_type="text/event-stream")

        chat_context = await context_switcher.determine_context_async(
            user_message=sanitized_message,
            selected_text=sanitized_selected_text
        )
        chat_context.memory = conversation_sessions.get(session_id)

        async def token_generator():
            try:
                current_response = ""
                is_first = True
                async for token in agent_wrapper.process_with_context_streamed(chat_context):
                    if is_first and token == OFF_TOPIC_SENTINEL:
                        yield json.dumps({
                            "type": "off_topic",
                            "content": "I can only answer questions about the book content. Please ask a question related to the book.",
                        }) + "\n"
                        return
                    is_first = False
                    current_response += token
                    yield json.dumps({"token": token}) + "\n"

                _update_memory(session_id, sanitized_message, current_response)
                yield json.dumps({"done": True, "session_id": session_id}) + "\n"
            except Exception as e:
                logger.error(f"Stream error: {str(e)}")
                yield json.dumps({"error": "Internal server error"}) + "\n"

        return StreamingResponse(
            token_generator(),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
                "X-Accel-Buffering": "no",
            },
        )

    except Exception as e:
        logger.error(f"Error setting up stream: {str(e)}")
        async def _error():
            yield json.dumps({"error": "Internal server error"}) + "\n"
        return StreamingResponse(_error(), media_type="text/event-stream")


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