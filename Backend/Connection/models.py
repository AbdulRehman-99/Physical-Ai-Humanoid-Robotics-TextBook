from pydantic import BaseModel
from typing import List, Optional, Dict, Any


class ChatRequest(BaseModel):
    """Represents a user's chat input with message and optional selected_text for context"""
    message: str
    selected_text: Optional[str] = None


class ChunkMetadata(BaseModel):
    """Contains book content metadata including module, chapter, section, and version information for proper attribution"""
    module: str
    chapter: str
    section: str
    version: str


class ChatResponse(BaseModel):
    """Represents the AI-generated response to the user's query, including sources with metadata (module, chapter, section, version)"""
    response: str
    sources: List[str] = []


class RetrievedChunk(BaseModel):
    """Represents a retrieved chunk with its content and metadata"""
    content: str
    metadata: Dict[str, Any] = {}
    score: float = 0.0


class ChatContext(BaseModel):
    """The resolved context provided to the Agent, either from selected_text or retrieved from Qdrant with metadata"""
    user_message: str
    retrieved_chunks: Optional[List[RetrievedChunk]] = None
    selected_text: Optional[str] = None
    effective_context: str