"""Data models for the ingestion system."""

from dataclasses import dataclass
from typing import Dict, List, Optional
from datetime import datetime

@dataclass
class DocumentMetadata:
    """Represents metadata about a source document."""

    module: str
    chapter: str
    section: str
    book_version: str
    file_path: str
    file_name: str
    created_date: Optional[datetime] = None
    last_modified: Optional[datetime] = None
    additional_metadata: Optional[Dict] = None

    def to_dict(self) -> Dict:
        """Convert to dictionary representation."""
        return {
            "module": self.module,
            "chapter": self.chapter,
            "section": self.section,
            "book_version": self.book_version,
            "file_path": self.file_path,
            "file_name": self.file_name,
            "created_date": self.created_date.isoformat() if self.created_date else None,
            "last_modified": self.last_modified.isoformat() if self.last_modified else None,
            "additional_metadata": self.additional_metadata or {}
        }

@dataclass
class DocumentChunk:
    """Represents a semantically coherent segment of text from a source document."""

    id: str
    text_content: str
    module: str
    chapter: str
    section: str
    book_version: str
    source_file_path: str
    chunk_index: int
    metadata: Optional[Dict] = None
    vector_embedding: Optional[List[float]] = None

    def to_dict(self) -> Dict:
        """Convert to dictionary representation for storage."""
        return {
            "id": self.id,
            "text_content": self.text_content,
            "module": self.module,
            "chapter": self.chapter,
            "section": self.section,
            "book_version": self.book_version,
            "source_file_path": self.source_file_path,
            "chunk_index": self.chunk_index,
            "metadata": self.metadata or {},
            "vector_embedding": self.vector_embedding
        }

@dataclass
class VectorEmbedding:
    """Represents a numerical vector representation of text content for semantic search."""

    chunk_id: str
    embedding: List[float]
    text_content: str
    metadata: Dict
    created_at: Optional[datetime] = None

    def to_dict(self) -> Dict:
        """Convert to dictionary representation for storage."""
        return {
            "id": self.chunk_id,
            "vector": self.embedding,
            "text_content": self.text_content,
            "metadata": self.metadata,
            "created_at": self.created_at.isoformat() if self.created_at else None
        }

@dataclass
class ProcessingResult:
    """Tracks the status and results of document processing."""

    document_path: str
    status: str  # "success", "failed", "partial"
    chunks_created: int = 0
    processing_time: float = 0.0  # in seconds
    error_message: Optional[str] = None
    processed_at: Optional[datetime] = None

    def to_dict(self) -> Dict:
        """Convert to dictionary representation."""
        return {
            "document_path": self.document_path,
            "status": self.status,
            "chunks_created": self.chunks_created,
            "processing_time": self.processing_time,
            "error_message": self.error_message,
            "processed_at": self.processed_at.isoformat() if self.processed_at else None
        }