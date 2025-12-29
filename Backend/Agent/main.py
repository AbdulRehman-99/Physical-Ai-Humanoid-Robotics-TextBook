from fastapi import FastAPI, HTTPException, Depends
from pydantic import BaseModel, Field
from typing import Optional, List
import sys
import os
from pathlib import Path

# Add the Retrieval/src directory to the path
retrieval_src_path = Path(__file__).parent.parent / "Retrieval" / "src"
sys.path.insert(0, str(retrieval_src_path))

import settings
from agent import RAGAgent

import health_monitor
import middleware
import logging_config
import logging
import asyncio


# Setup comprehensive logging
logger = logging.getLogger(__name__)

app = FastAPI(
    title="RAG Agent for Book Content",
    description="API for the RAG Agent that answers user questions from book content",
    version="1.0.0"
)

# Add rate limiting middleware
app.add_middleware(middleware.RateLimitMiddleware)

# Add error logging middleware
logging_config.add_error_logging_middleware(app)


class ChatPayload(BaseModel):
    message: str = Field(..., description="The user's message or question")
    selected_text: Optional[str] = Field(
        None,
        description="Optional selected text from the UI to use as context"
    )


class ChatResponse(BaseModel):
    response: str = Field(..., description="The agent's response to the user's query")
    source_chunks: List[str] = Field(
        default_factory=list,
        description="The source chunks used to generate the response (when applicable)"
    )
    confidence: float = Field(
        0.0,
        ge=0.0,
        le=1.0,
        description="Confidence score of the response"
    )


class ErrorResponse(BaseModel):
    error: str = Field(..., description="Error message")


@app.on_event("startup")
async def startup_event():
    logger.info("Starting up RAG Agent API")

    # Initialize health monitoring in the background
    import asyncio
    monitor = health_monitor.get_health_monitor()
    # Run the health monitoring as a background task
    asyncio.create_task(monitor.periodic_health_check())


@app.on_event("shutdown")
async def shutdown_event():
    logger.info("Shutting down RAG Agent API")


@app.post("/chat", response_model=ChatResponse, responses={
    400: {"model": ErrorResponse},
    500: {"model": ErrorResponse}
})
async def chat(payload: ChatPayload):
    """
    Send a message to the RAG agent and receive a response based on book content or selected text
    """
    try:
        # Validate character limit for selected_text
        if payload.selected_text and len(payload.selected_text) > 10000:
            raise HTTPException(
                status_code=400,
                detail="Selected text exceeds 10,000 character limit"
            )

        # Initialize the RAG Agent
        agent = RAGAgent()

        # Get response from the agent
        response = await agent.process_message(
            message=payload.message,
            selected_text=payload.selected_text
        )

        return ChatResponse(
            response=response.response,
            source_chunks=response.source_chunks,
            confidence=response.confidence
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing chat request: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail="Internal server error occurred while processing the request"
        )


@app.get("/health")
async def health_check():
    """
    Overall health check endpoint
    """
    return {"status": "healthy"}


@app.get("/health/database")
async def database_health():
    """
    Vector database health check endpoint
    """
    monitor = health_monitor.get_health_monitor()
    health_status = await monitor.check_qdrant_health()
    return health_status


@app.get("/health/status")
async def full_health_status():
    """
    Get comprehensive health status including all components
    """
    monitor = health_monitor.get_health_monitor()
    db_health = await monitor.check_qdrant_health()

    return {
        "overall_status": "healthy" if db_health["connected"] else "unhealthy",
        "timestamp": __import__('time').time(),
        "components": {
            "vector_database": db_health
        }
    }


if __name__ == "__main__":
    import uvicorn
    # Use a different port to avoid conflict with main backend
    port = int(os.getenv("PORT", 8001))
    uvicorn.run(
        "main:app",
        host=settings.settings.host,
        port=port,
        reload=settings.settings.debug
    )
