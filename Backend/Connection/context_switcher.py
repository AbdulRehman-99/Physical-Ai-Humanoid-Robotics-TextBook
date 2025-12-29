from typing import Optional, Dict, Any, List
from .models import ChatContext
import sys
import os
from pathlib import Path

# Add the Retrieval/src directory to the path to import the working retrieval system
retrieval_src_path = Path(__file__).parent.parent / "Retrieval" / "src"
sys.path.insert(0, str(retrieval_src_path))

from retrieval import get_retrieval_service
import logging

logger = logging.getLogger(__name__)

class ContextSwitcher:
    def __init__(self):
        self.retrieval_service = get_retrieval_service()

    def determine_context(self, user_message: str, selected_text: Optional[str] = None) -> ChatContext:
        """
        Implement logic to choose between selected_text and Qdrant retrieval
        Ensure selected_text always overrides vector retrieval when present
        """
        if selected_text and selected_text.strip():
            # If selected_text exists, use it as context and bypass Qdrant retrieval
            effective_context = selected_text
            retrieved_chunks = None
        else:
            # If no selected_text, retrieve from Qdrant
            # Using the retrieval service to get relevant chunks
            try:
                # Use the retrieval service to get search results
                search_results = self.retrieval_service.search_similar(user_message, top_k=5)
                if search_results:
                    # Convert search results to the expected format for retrieved_chunks
                    retrieved_chunks = []
                    context_parts = []
                    for result in search_results:
                        chunk_text = result.get('content', '')
                        if chunk_text and chunk_text.strip():
                            # Extract metadata fields expected by response_formatter
                            result_metadata = result.get('metadata', {})
                            formatted_metadata = {
                                "module": result_metadata.get('module', 'N/A'),
                                "chapter": result_metadata.get('chapter', 'N/A'),
                                "section": result_metadata.get('section', 'N/A'),
                                "version": result_metadata.get('version', 'N/A'),
                                "score": result.get('score', 0.0),  # similarity score
                                "original_metadata": result_metadata  # keep original for reference
                            }
                            retrieved_chunks.append({
                                "text": chunk_text,
                                "metadata": formatted_metadata
                            })
                            context_parts.append(chunk_text)

                    # Combine all chunk texts for effective_context
                    effective_context = " ".join(context_parts)
                else:
                    # If no relevant chunks found in Qdrant, return appropriate response indicating no relevant content found
                    retrieved_chunks = None
                    effective_context = f"No relevant content found in the book for the query: {user_message}"
            except Exception as e:
                logger.error(f"Error retrieving from Qdrant using retrieval service: {str(e)}")
                retrieved_chunks = None
                effective_context = f"No relevant content found in the book for the query: {user_message}"

        return ChatContext(
            user_message=user_message,
            retrieved_chunks=retrieved_chunks,
            selected_text=selected_text,
            effective_context=effective_context
        )

    async def determine_context_async(self, user_message: str, selected_text: Optional[str] = None) -> ChatContext:
        """
        Async version that properly handles Qdrant retrieval
        """
        if selected_text and selected_text.strip():
            # If selected_text exists, use it as context and bypass Qdrant retrieval
            effective_context = selected_text
            retrieved_chunks = None
        else:
            # If no selected_text, retrieve from Qdrant using the proper retrieval service
            try:
                # Use search_similar to get individual results that can be formatted properly
                search_results = self.retrieval_service.search_similar(user_message, top_k=5)
                if search_results:
                    # Convert search results to the expected format for retrieved_chunks
                    retrieved_chunks = []
                    context_parts = []
                    for result in search_results:
                        chunk_text = result.get('content', '')
                        if chunk_text and chunk_text.strip():
                            # Extract metadata fields expected by response_formatter
                            result_metadata = result.get('metadata', {})
                            formatted_metadata = {
                                "module": result_metadata.get('module', 'N/A'),
                                "chapter": result_metadata.get('chapter', 'N/A'),
                                "section": result_metadata.get('section', 'N/A'),
                                "version": result_metadata.get('version', 'N/A'),
                                "score": result.get('score', 0.0),  # similarity score
                                "original_metadata": result_metadata  # keep original for reference
                            }
                            retrieved_chunks.append({
                                "text": chunk_text,
                                "metadata": formatted_metadata
                            })
                            context_parts.append(chunk_text)

                    # Combine all chunk texts for effective_context
                    effective_context = " ".join(context_parts)
                else:
                    # If no relevant chunks found in Qdrant, return appropriate response indicating no relevant content found
                    retrieved_chunks = None
                    effective_context = f"No relevant content found in the book for the query: {user_message}"
            except Exception as e:
                logger.error(f"Error retrieving from Qdrant using retrieval service: {str(e)}")
                retrieved_chunks = None
                effective_context = f"No relevant content found in the book for the query: {user_message}"

        return ChatContext(
            user_message=user_message,
            retrieved_chunks=retrieved_chunks,
            selected_text=selected_text,
            effective_context=effective_context
        )