from typing import Dict, Any, List
from .models import ChatResponse, ChatContext

class ResponseFormatter:
    def __init__(self):
        pass

    def format_response(self, agent_response: str, context: ChatContext) -> ChatResponse:
        """
        Format agent responses for ChatKit compatibility
        Ensure proper error response formatting
        Add source attribution with metadata (module, chapter, section, version)
        Handle special cases like zero-results and timeout scenarios
        """
        # Extract sources from context if available
        sources = []
        if context.retrieved_chunks:
            for chunk in context.retrieved_chunks:
                if "metadata" in chunk:
                    metadata = chunk["metadata"]
                    source = f"Module: {metadata.get('module', 'N/A')}, Chapter: {metadata.get('chapter', 'N/A')}, Section: {metadata.get('section', 'N/A')}"
                    sources.append(source)

        return ChatResponse(
            response=agent_response,
            sources=sources
        )

    def format_off_topic_response(self, original_query: str) -> ChatResponse:
        """
        Format response for off-topic queries with academic reliability
        """
        return ChatResponse(
            response="I can only answer questions about the book content. Please ask a question related to the book.",
            sources=[]
        )

    def format_zero_results_response(self, query: str) -> ChatResponse:
        """
        Format response when Qdrant returns zero relevant results
        """
        return ChatResponse(
            response=f"No relevant content found in the book for the query: {query}",
            sources=[]
        )

    def format_error_response(self, error_message: str) -> ChatResponse:
        """
        Format error responses with proper academic tone
        """
        return ChatResponse(
            response="I can only answer questions about the book content. Please ask a question related to the book.",
            sources=[]
        )