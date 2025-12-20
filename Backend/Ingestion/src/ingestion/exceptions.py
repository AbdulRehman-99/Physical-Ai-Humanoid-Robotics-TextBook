"""Custom exception classes for the ingestion system."""

class IngestionError(Exception):
    """Base exception class for ingestion-related errors."""
    pass


class DocumentProcessingError(IngestionError):
    """Raised when there's an error processing a document."""
    def __init__(self, message: str, document_path: str = None):
        super().__init__(message)
        self.document_path = document_path


class ConfigurationError(IngestionError):
    """Raised when there's an error with configuration."""
    pass


class APIClientError(IngestionError):
    """Raised when there's an error with API client operations."""
    def __init__(self, message: str, api_name: str = None):
        super().__init__(message)
        self.api_name = api_name


class VectorStorageError(IngestionError):
    """Raised when there's an error with vector storage operations."""
    def __init__(self, message: str, operation: str = None):
        super().__init__(message)
        self.operation = operation


class TextProcessingError(IngestionError):
    """Raised when there's an error during text processing."""
    pass


class ChunkingError(IngestionError):
    """Raised when there's an error during text chunking."""
    pass


class ValidationError(IngestionError):
    """Raised when validation fails."""
    pass