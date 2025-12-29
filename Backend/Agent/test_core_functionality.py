"""
Comprehensive tests for the RAG Agent core functionality
"""
import pytest
import asyncio
from unittest.mock import Mock, patch, AsyncMock
from .agent import RAGAgent, AgentResponse, ContextSource
from .retrieval import RetrievalService
from .adapter import GeminiAdapter
from .validation import validate_response_quality


class TestRAGAgent:
    """Test cases for the RAG Agent"""

    @pytest.fixture
    def agent(self):
        """Create a test agent instance"""
        with patch('Backend.Agent.agent.get_gemini_client') as mock_gemini, \
             patch('Backend.Agent.agent.get_retrieval_service') as mock_retrieval:
            mock_gemini.return_value = Mock(spec=GeminiAdapter)
            mock_retrieval.return_value = Mock(spec=RetrievalService)
            return RAGAgent()

    @pytest.mark.asyncio
    async def test_process_message_with_selected_text(self, agent):
        """Test processing a message with selected text context"""
        # Mock the generate_response method
        with patch.object(agent, 'generate_response', new_callable=AsyncMock) as mock_gen:
            mock_gen.return_value = "Test response based on selected text"

            result = await agent.process_message(
                message="What does this text mean?",
                selected_text="This is the selected text that should be used as context."
            )

            # Verify the result
            assert isinstance(result, AgentResponse)
            assert result.response == "Test response based on selected text"
            assert result.source_chunks == ["This is the selected text that should be used as context."]

            # Verify that generate_response was called with correct context
            mock_gen.assert_called_once()
            args, kwargs = mock_gen.call_args
            context_source = args[1]  # Second argument is context_source
            assert context_source.type == "selected_text"
            assert "This is the selected text" in context_source.content

    @pytest.mark.asyncio
    async def test_process_message_with_vector_retrieval(self, agent):
        """Test processing a message with vector retrieval context"""
        # Mock the retrieval service
        agent.retrieval_service.get_book_context = Mock(return_value="Retrieved book content")

        # Mock the generate_response method
        with patch.object(agent, 'generate_response', new_callable=AsyncMock) as mock_gen:
            mock_gen.return_value = "Test response based on retrieved content"

            result = await agent.process_message(
                message="What does this topic mean?",
                selected_text=None  # No selected text, so use vector retrieval
            )

            # Verify the result
            assert isinstance(result, AgentResponse)
            assert result.response == "Test response based on retrieved content"
            assert result.source_chunks == ["Retrieved book content"]

            # Verify that generate_response was called with correct context
            mock_gen.assert_called_once()
            args, kwargs = mock_gen.call_args
            context_source = args[1]  # Second argument is context_source
            assert context_source.type == "vector_retrieval"
            assert "Retrieved book content" in context_source.content

    @pytest.mark.asyncio
    async def test_process_message_fallback_when_db_unavailable(self, agent):
        """Test fallback behavior when vector database is unavailable"""
        # Mock the retrieval service to raise an exception
        agent.retrieval_service.get_book_context = Mock(side_effect=Exception("Database unavailable"))

        # Mock the generate_response method
        with patch.object(agent, 'generate_response', new_callable=AsyncMock) as mock_gen:
            mock_gen.return_value = "Test response with fallback logic"

            result = await agent.process_message(
                message="What does this topic mean?",
                selected_text=None  # No selected text, so try vector retrieval (which will fail)
            )

            # Verify the result
            assert isinstance(result, AgentResponse)
            assert result.response == "Test response with fallback logic"
            assert result.source_chunks == [""]  # Empty context in fallback
            assert result.confidence == 0.5  # Lower confidence for fallback

            # Verify that generate_response was called with fallback context
            mock_gen.assert_called_once()
            args, kwargs = mock_gen.call_args
            context_source = args[1]  # Second argument is context_source
            assert context_source.type == "llm_only"
            assert context_source.content == ""


class TestValidation:
    """Test cases for validation functionality"""

    def test_validate_academic_reliability(self):
        """Test academic reliability validation"""
        response = "The theory of relativity was developed by Albert Einstein."
        context = "Einstein developed the theory of relativity in the early 1900s."

        result = validate_response_quality(response, context)

        assert "is_reliable" in result
        assert "reliability_score" in result
        assert "issues" in result
        assert isinstance(result["is_reliable"], bool)

    def test_validate_response_quality_with_no_context(self):
        """Test validation with empty context"""
        response = "The theory of relativity was developed by Albert Einstein."
        context = ""

        result = validate_response_quality(response, context)

        # Should handle empty context gracefully
        assert "is_reliable" in result
        assert "reliability_score" in result


class TestRetrievalService:
    """Test cases for retrieval service"""

    @pytest.fixture
    def retrieval_service(self):
        """Create a test retrieval service instance"""
        with patch('qdrant_client.QdrantClient') as mock_qdrant, \
             patch('cohere.Client') as mock_cohere:
            return RetrievalService()

    def test_embed_text(self, retrieval_service):
        """Test text embedding functionality"""
        # Mock the cohere client
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3]]
        retrieval_service.cohere_client.embed = Mock(return_value=mock_response)

        result = retrieval_service.embed_text("Test text for embedding")

        assert result == [0.1, 0.2, 0.3]
        retrieval_service.cohere_client.embed.assert_called_once()


def test_error_handling_decorator():
    """Test that error handling decorator works correctly"""
    from .error_handling import handle_qdrant_errors, QdrantError

    @handle_qdrant_errors
    def test_function():
        raise Exception("Test error")

    try:
        test_function()
        assert False, "Expected QdrantError to be raised"
    except QdrantError:
        # Expected behavior
        pass


if __name__ == "__main__":
    pytest.main([__file__])