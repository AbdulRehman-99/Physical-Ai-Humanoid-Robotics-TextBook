from typing import Optional, Dict, Any, List
from .models import ChatContext, RetrievedChunk
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
                    logger.info(f"Raw search results structure: {type(search_results)}, count: {len(search_results) if isinstance(search_results, list) else 'N/A'}")
                    if search_results and len(search_results) > 0:
                        logger.info(f"First result keys: {list(search_results[0].keys()) if isinstance(search_results[0], dict) else 'Not a dict'}")

                    for result in search_results:
                        # Check the actual structure of the result
                        # The result from the retrieval service should have 'content', 'score', 'metadata' as top-level keys
                        if isinstance(result, dict):
                            # The content should be directly accessible as 'content' field
                            chunk_text = result.get('content', '')

                            # If the content is still empty, try to get it from payload (backup approach)
                            if not chunk_text.strip():
                                chunk_text = result.get('payload', {}).get('content', '') or result.get('payload', {}).get('text', '')

                            # Also try other possible locations
                            if not chunk_text.strip():
                                chunk_text = result.get('text', '')

                            # Get the metadata and score
                            metadata = result.get('metadata', result.get('payload', {}))
                            score = result.get('score', 0.0)
                        else:
                            # If it's not a dict, try to handle it differently
                            chunk_text = str(result) if hasattr(result, '__str__') else ''
                            metadata = {}
                            score = 0.0

                        if chunk_text and chunk_text.strip():
                            # For ChatContext model, retrieved_chunks should be a list of RetrievedChunk objects
                            chunk_obj = RetrievedChunk(content=chunk_text, metadata=metadata, score=score)
                            retrieved_chunks.append(chunk_obj)
                            context_parts.append(chunk_text)
                        else:
                            logger.warning(f"Found result without content: {result}")

                    # Combine all chunk texts for effective_context
                    effective_context = " ".join(context_parts)
                    logger.info(f"Retrieved {len(retrieved_chunks)} chunks with total content length: {len(effective_context)} chars")
                    if retrieved_chunks:
                        logger.info(f"First retrieved chunk preview: {retrieved_chunks[0].content[:200]}...")
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
                    logger.info(f"Raw search results structure: {type(search_results)}, count: {len(search_results) if isinstance(search_results, list) else 'N/A'}")
                    if search_results and len(search_results) > 0:
                        logger.info(f"First result keys: {list(search_results[0].keys()) if isinstance(search_results[0], dict) else 'Not a dict'}")

                    for result in search_results:
                        # Check the actual structure of the result
                        # The result from the retrieval service should have 'content', 'score', 'metadata' as top-level keys
                        if isinstance(result, dict):
                            # The content should be directly accessible as 'content' field
                            chunk_text = result.get('content', '')

                            # If the content is still empty, try to get it from payload (backup approach)
                            if not chunk_text.strip():
                                chunk_text = result.get('payload', {}).get('content', '') or result.get('payload', {}).get('text', '')

                            # Also try other possible locations
                            if not chunk_text.strip():
                                chunk_text = result.get('text', '')

                            # Get the metadata and score
                            metadata = result.get('metadata', result.get('payload', {}))
                            score = result.get('score', 0.0)
                        else:
                            # If it's not a dict, try to handle it differently
                            chunk_text = str(result) if hasattr(result, '__str__') else ''
                            metadata = {}
                            score = 0.0

                        if chunk_text and chunk_text.strip():
                            # For ChatContext model, retrieved_chunks should be a list of RetrievedChunk objects
                            chunk_obj = RetrievedChunk(content=chunk_text, metadata=metadata, score=score)
                            retrieved_chunks.append(chunk_obj)
                            context_parts.append(chunk_text)
                        else:
                            logger.warning(f"Found result without content: {result}")

                    # Combine all chunk texts for effective_context
                    effective_context = " ".join(context_parts)
                    logger.info(f"Retrieved {len(retrieved_chunks)} chunks with total content length: {len(effective_context)} chars")
                    if retrieved_chunks:
                        logger.info(f"First retrieved chunk preview: {retrieved_chunks[0].content[:200]}...")
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