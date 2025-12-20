"""Module for processing markdown documents and extracting content and metadata."""

import os
import re
from pathlib import Path
from typing import List, Dict, Tuple, Optional
from markdown import markdown
import frontmatter  # For parsing markdown with YAML frontmatter
from src.ingestion.models import DocumentChunk, DocumentMetadata, ProcessingResult
from src.ingestion.utils import get_file_size, calculate_file_hash
from src.ingestion.exceptions import DocumentProcessingError, ValidationError
from src.config.settings import settings
from src.ingestion.text_cleaner import TextCleaner
from src.ingestion.chunker import Chunker
import logging
from datetime import datetime

logger = logging.getLogger(__name__)

class DocumentProcessor:
    """Class to handle the processing of markdown documents."""

    def __init__(self):
        """Initialize the document processor."""
        self.max_document_size = settings.max_document_size  # From settings
        self.text_cleaner = TextCleaner()  # Initialize text cleaner
        self.chunker = Chunker()  # Initialize chunker with default settings from config

    def scan_documents(self, base_path: str) -> List[Path]:
        """
        Scan and return all markdown files in the given base path.

        Args:
            base_path: The directory path to scan for markdown files

        Returns:
            List of Path objects for all .md files found
        """
        from src.ingestion.utils import get_all_markdown_files
        return get_all_markdown_files(base_path)

    def extract_metadata_from_path(self, file_path: Path) -> DocumentMetadata:
        """
        Extract metadata from the file path structure.
        Expected structure: Frontend/docs/module/chapter/section.md or similar patterns.

        Args:
            file_path: Path to the markdown file

        Returns:
            DocumentMetadata object with extracted information
        """
        # Convert to relative path from Frontend/docs
        frontend_docs_path = Path("Frontend/docs")
        try:
            # Find the relative path from Frontend/docs
            relative_parts = []
            path_parts = file_path.parts

            # Look for 'docs' in the path to start from there
            docs_index = -1
            for i, part in enumerate(path_parts):
                if part == 'docs':
                    docs_index = i
                    break

            if docs_index != -1:
                relative_parts = path_parts[docs_index + 1:]  # Skip 'docs'
            else:
                # If 'docs' not found, use the whole path relative to file name
                relative_parts = path_parts[path_parts.rindex('Frontend') + 2:] if 'Frontend' in path_parts else path_parts[-2:]

            # Extract components based on path structure
            if len(relative_parts) >= 2:
                module = relative_parts[0] if relative_parts[0] else "unknown_module"
                chapter = relative_parts[1].replace('.md', '') if relative_parts[1] else "unknown_chapter"

                # Handle the case where there's a section or more specific part
                if len(relative_parts) >= 3:
                    section = relative_parts[2].replace('.md', '') if relative_parts[2] else chapter
                else:
                    section = chapter  # Use chapter name as section if no specific section
            else:
                module = "unknown_module"
                chapter = file_path.stem
                section = file_path.stem

            # Default book version - in a real implementation, this might come from a config or file
            book_version = "v1.0"

            # Get file stats
            stat = file_path.stat()
            created_date = datetime.fromtimestamp(stat.st_ctime)
            last_modified = datetime.fromtimestamp(stat.st_mtime)

            return DocumentMetadata(
                module=module,
                chapter=chapter,
                section=section,
                book_version=book_version,
                file_path=str(file_path),
                file_name=file_path.name,
                created_date=created_date,
                last_modified=last_modified
            )
        except Exception as e:
            logger.warning(f"Could not extract metadata from path {file_path}: {e}")
            # Return default metadata if path parsing fails
            return DocumentMetadata(
                module="unknown_module",
                chapter="unknown_chapter",
                section="unknown_section",
                book_version="v1.0",
                file_path=str(file_path),
                file_name=file_path.name
            )

    def read_and_parse_document(self, file_path: Path) -> Tuple[str, Dict]:
        """
        Read and parse a markdown document, extracting content and frontmatter.

        Args:
            file_path: Path to the markdown file

        Returns:
            Tuple of (content string, metadata dictionary from frontmatter)
        """
        try:
            # Check file size first
            size = get_file_size(file_path)
            if size > self.max_document_size:
                raise DocumentProcessingError(
                    f"Document size {size} exceeds maximum allowed size {self.max_document_size}",
                    document_path=str(file_path)
                )

            # Read file content
            with open(file_path, 'r', encoding='utf-8') as file:
                content = file.read()

            # Parse frontmatter if present
            try:
                post = frontmatter.loads(content)
                content = post.content
                metadata = post.metadata
            except Exception:
                # If frontmatter parsing fails, just return content with empty metadata
                metadata = {}
                # We already have content from the file read

            return content, metadata

        except UnicodeDecodeError:
            # Try different encoding
            try:
                with open(file_path, 'r', encoding='latin-1') as file:
                    content = file.read()
                return content, {}
            except Exception as e:
                raise DocumentProcessingError(
                    f"Failed to read file with any encoding: {e}",
                    document_path=str(file_path)
                )
        except Exception as e:
            raise DocumentProcessingError(
                f"Failed to read document: {e}",
                document_path=str(file_path)
            )

    def validate_document_content(self, content: str, file_path: Path) -> bool:
        """
        Validate the content of a document.

        Args:
            content: The document content to validate
            file_path: Path to the document (for error reporting)

        Returns:
            True if content is valid, raises exception if not
        """
        if not content or not content.strip():
            raise ValidationError(f"Document {file_path} has no content", file_path=str(file_path))

        # Check for excessive null bytes or other problematic content
        if content.count('\x00') / len(content) > 0.1:  # More than 10% null bytes
            raise ValidationError(f"Document {file_path} appears to be binary or corrupted", file_path=str(file_path))

        return True

    def process_single_document(self, file_path: Path) -> List[DocumentChunk]:
        """
        Process a single document: read, parse, clean, and convert to chunks.

        Args:
            file_path: Path to the markdown file

        Returns:
            List of DocumentChunk objects
        """
        start_time = datetime.now()

        try:
            # Extract metadata from path
            doc_metadata = self.extract_metadata_from_path(file_path)

            # Read and parse the document
            content, frontmatter_meta = self.read_and_parse_document(file_path)

            # Clean and normalize the content
            cleaned_content = self.text_cleaner.clean_text(content)

            # Validate content
            self.validate_document_content(cleaned_content, file_path)

            # Combine path metadata with frontmatter metadata
            combined_metadata = {**doc_metadata.to_dict(), **frontmatter_meta}

            # Create an initial chunk with the full content
            initial_chunk = DocumentChunk(
                id=f"{file_path}_0",
                text_content=cleaned_content,
                module=doc_metadata.module,
                chapter=doc_metadata.chapter,
                section=doc_metadata.section,
                book_version=doc_metadata.book_version,
                source_file_path=str(file_path),
                chunk_index=0,
                metadata=combined_metadata
            )

            # Use the chunker to create semantic chunks if the content is too large
            document_chunks = [initial_chunk]
            final_chunks = self.chunker.chunk_document_content(document_chunks)

            processing_time = (datetime.now() - start_time).total_seconds()
            logger.info(f"Successfully processed document {file_path} in {processing_time:.2f}s, created {len(final_chunks)} semantic chunks")

            return final_chunks

        except Exception as e:
            processing_time = (datetime.now() - start_time).total_seconds()
            logger.error(f"Failed to process document {file_path}: {e}")

            # Still return a processing result even if processing failed
            raise e

    def process_documents(self, base_path: str) -> List[ProcessingResult]:
        """
        Process all documents in the given base path.

        Args:
            base_path: The directory path containing markdown files

        Returns:
            List of ProcessingResult objects tracking the results
        """
        logger.info(f"Starting to process documents in {base_path}")

        # Get all markdown files
        markdown_files = self.scan_documents(base_path)
        logger.info(f"Found {len(markdown_files)} markdown files to process")

        results = []
        processed_count = 0
        failed_count = 0

        for file_path in markdown_files:
            start_time = datetime.now()
            try:
                # Process each document
                chunks = self.process_single_document(file_path)

                # Calculate processing time
                processing_time = (datetime.now() - start_time).total_seconds()

                # Create processing result
                result = ProcessingResult(
                    document_path=str(file_path),
                    status="success",
                    chunks_created=len(chunks),
                    processing_time=processing_time,
                    processed_at=datetime.now()
                )

                results.append(result)
                processed_count += 1

                logger.info(f"Processed {file_path} successfully in {processing_time:.2f}s")

            except Exception as e:
                processing_time = (datetime.now() - start_time).total_seconds()
                logger.error(f"Failed to process {file_path}: {str(e)}")

                # Create failure result
                result = ProcessingResult(
                    document_path=str(file_path),
                    status="failed",
                    chunks_created=0,
                    processing_time=processing_time,
                    error_message=str(e),
                    processed_at=datetime.now()
                )

                results.append(result)
                failed_count += 1

        logger.info(f"Document processing completed. {processed_count} successful, {failed_count} failed")
        return results

def create_ingestion_pipeline():
    """
    Create a basic ingestion pipeline function that orchestrates the document processing.

    Returns:
        A function that can be called to run the ingestion pipeline
    """
    def run_ingestion_pipeline(source_path: str = "Frontend/docs", target_collection: str = None):
        """
        Run the complete ingestion pipeline.

        Args:
            source_path: Path to the source documents (default: "Frontend/docs")
            target_collection: Optional target collection name (uses default if None)
        """
        logger.info("Starting ingestion pipeline...")

        # Initialize components
        processor = DocumentProcessor()

        # Process documents
        results = processor.process_documents(source_path)

        # Calculate summary statistics
        total_docs = len(results)
        successful_docs = len([r for r in results if r.status == "success"])
        failed_docs = len([r for r in results if r.status == "failed"])
        total_chunks = sum(r.chunks_created for r in results)

        logger.info(f"Ingestion pipeline completed!")
        logger.info(f"Summary: {successful_docs}/{total_docs} documents processed successfully")
        logger.info(f"Total chunks created: {total_chunks}")
        if failed_docs > 0:
            logger.warning(f"Failed to process {failed_docs} documents")

        return results

    return run_ingestion_pipeline