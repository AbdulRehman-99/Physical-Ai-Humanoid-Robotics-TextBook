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
                # chunk is now a RetrievedChunk object with metadata
                metadata = chunk.metadata
                if metadata:
                    module = metadata.get('module', 'N/A')
                    chapter = metadata.get('chapter', 'N/A')
                    section = metadata.get('section', 'N/A')
                    source = f"Module: {module}, Chapter: {chapter}, Section: {section}"
                    sources.append(source)
                else:
                    sources.append("Book content retrieved")

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