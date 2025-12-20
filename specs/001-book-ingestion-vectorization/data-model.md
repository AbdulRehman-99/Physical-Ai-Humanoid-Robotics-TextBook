# Data Model: Book Ingestion & Vectorization

## Document Chunk Entity

**Definition**: Represents a semantically coherent segment of text from a source document

**Attributes**:
- `id` (string): Unique identifier for the chunk
- `text_content` (string): The actual text content of the chunk
- `module` (string): Module identifier from source document
- `chapter` (string): Chapter number/name from source document
- `section` (string): Section title from source document
- `book_version` (string): Version of the book
- `source_file_path` (string): Path to the original markdown file
- `chunk_index` (integer): Position of this chunk within the original document
- `vector_embedding` (list[float]): Numerical vector representation of the text content
- `metadata` (dict): Additional metadata preserved from source document

## Document Metadata Entity

**Definition**: Contains information about the source document

**Attributes**:
- `module` (string): Module identifier (e.g., "Module-1-ROS")
- `chapter` (string): Chapter number/name (e.g., "Chapter-1-Foundations")
- `section` (string): Section title (e.g., "Introduction to ROS2")
- `book_version` (string): Version identifier (e.g., "v1.0")
- `file_path` (string): Path to the markdown file
- `file_name` (string): Name of the markdown file
- `created_date` (datetime): When the file was created
- `last_modified` (datetime): When the file was last modified

## Vector Embedding Entity

**Definition**: Numerical representation of text content for semantic search, stored in Qdrant vector database

**Attributes**:
- `id` (string): Unique identifier matching the document chunk
- `vector` (list[float]): High-dimensional vector representation from Cohere (stored as Qdrant vector payload)
- `text_content` (string): Original text that was embedded (stored as Qdrant payload)
- `metadata` (dict): Associated metadata for retrieval context (stored as Qdrant payload with fields: module, chapter, section, book_version, source_file_path)
- `created_at` (datetime): When the embedding was generated (stored as Qdrant payload)

## Processing Result Entity

**Definition**: Tracks the status and results of document processing

**Attributes**:
- `document_path` (string): Path of the processed document
- `status` (string): Processing status (success, failed, partial)
- `chunks_created` (integer): Number of chunks created from this document
- `processing_time` (float): Time taken to process the document (seconds)
- `error_message` (string): Error details if processing failed
- `processed_at` (datetime): When processing was completed