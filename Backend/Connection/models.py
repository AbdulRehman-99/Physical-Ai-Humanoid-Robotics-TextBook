from pydantic import BaseModel
from typing import List, Optional


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


class ChatContext(BaseModel):
    """The resolved context provided to the Agent, either from selected_text or retrieved from Qdrant with metadata"""
    user_message: str
    retrieved_chunks: Optional[List[str]] = None
    selected_text: Optional[str] = None
    effective_context: str