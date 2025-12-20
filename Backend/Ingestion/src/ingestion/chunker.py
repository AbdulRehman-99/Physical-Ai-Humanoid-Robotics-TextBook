"""Module for semantic chunking of text with overlap."""

import re
from typing import List, Tuple, Optional
from dataclasses import dataclass
from src.ingestion.models import DocumentChunk
from src.config.settings import settings
import logging

logger = logging.getLogger(__name__)

@dataclass
class Chunk:
    """Represents a text chunk with metadata."""
    id: str
    text: str
    index: int
    metadata: dict


class Chunker:
    """Class to handle semantic chunking of text with configurable size and overlap."""

    def __init__(self, chunk_size: int = None, overlap_size: int = None):
        """
        Initialize the chunker with configurable parameters.

        Args:
            chunk_size: Size of each chunk in tokens (default from settings)
            overlap_size: Size of overlap between chunks in tokens (default from settings)
        """
        self.chunk_size = chunk_size or settings.chunk_size
        self.overlap_size = overlap_size or settings.chunk_overlap

        # Validate parameters
        if self.overlap_size >= self.chunk_size:
            raise ValueError(f"Overlap size ({self.overlap_size}) must be less than chunk size ({self.chunk_size})")

    def tokenize(self, text: str) -> List[str]:
        """
        Simple tokenization by splitting on whitespace.
        In a production system, you might use a more sophisticated tokenizer.

        Args:
            text: Input text

        Returns:
            List of tokens
        """
        # Split on whitespace but preserve sentence boundaries for better semantic chunks
        # First, add markers for sentence endings
        text = re.sub(r'([.!?])\s+', r'\1 <EOS> ', text)
        tokens = text.split()
        return tokens

    def detokenize(self, tokens: List[str]) -> str:
        """
        Convert tokens back to text, handling sentence boundaries.

        Args:
            tokens: List of tokens

        Returns:
            Reconstructed text
        """
        text = ' '.join(tokens)
        # Restore sentence endings
        text = re.sub(r'\s*<EOS>\s*', '. ', text)
        return text.strip()

    def identify_semantic_boundaries(self, tokens: List[str]) -> List[int]:
        """
        Identify semantic boundaries in the token list (e.g., at sentence ends).

        Args:
            tokens: List of tokens

        Returns:
            List of indices where semantic breaks occur
        """
        boundaries = []
        for i, token in enumerate(tokens):
            if token == '<EOS>':
                boundaries.append(i)
        return boundaries

    def create_semantic_chunks(self, text: str, doc_metadata: dict = None) -> List[Chunk]:
        """
        Create semantic chunks from text with configurable size and overlap.

        Args:
            text: Input text to chunk
            doc_metadata: Metadata from the original document

        Returns:
            List of Chunk objects
        """
        if not text.strip():
            return []

        # Tokenize the text
        tokens = self.tokenize(text)

        if len(tokens) <= self.chunk_size:
            # If text is smaller than chunk size, return as single chunk
            return [Chunk(
                id=f"chunk_0",
                text=self.detokenize(tokens),
                index=0,
                metadata=doc_metadata or {}
            )]

        chunks = []
        start_idx = 0
        chunk_idx = 0

        while start_idx < len(tokens):
            # Determine the end index for this chunk
            end_idx = min(start_idx + self.chunk_size, len(tokens))

            # Try to find a semantic boundary within the last 20% of the chunk
            # to avoid breaking sentences in the middle
            search_start = max(start_idx, end_idx - max(10, self.chunk_size // 5))
            semantic_break = -1

            for i in range(end_idx - 1, search_start - 1, -1):
                if i < len(tokens) and tokens[i] == '<EOS>':
                    semantic_break = i + 1  # Include the EOS token
                    break

            # Use semantic break if found and it's not too far from the desired end
            if semantic_break != -1 and semantic_break > start_idx + self.chunk_size // 2:
                actual_end = semantic_break
            else:
                actual_end = end_idx

            # Extract tokens for this chunk
            chunk_tokens = tokens[start_idx:actual_end]
            chunk_text = self.detokenize(chunk_tokens)

            # Create chunk
            chunk = Chunk(
                id=f"chunk_{chunk_idx}",
                text=chunk_text,
                index=chunk_idx,
                metadata=doc_metadata or {}
            )
            chunks.append(chunk)

            # Determine next start index with overlap
            if actual_end >= len(tokens):
                # Reached the end
                break

            # Calculate next start position with overlap
            # Move back by overlap amount but ensure we don't go backwards
            next_start = max(
                actual_end - self.overlap_size,  # Apply overlap
                start_idx + 1  # Ensure we make progress
            )

            # If overlap would cause us to repeat the same text, move forward by chunk_size
            if next_start <= start_idx:
                next_start = start_idx + self.chunk_size

            start_idx = next_start
            chunk_idx += 1

        logger.info(f"Created {len(chunks)} chunks from text of length {len(tokens)} tokens")
        return chunks

    def validate_chunk_parameters(self, chunk_size: int, overlap_size: int) -> bool:
        """
        Validate that chunk parameters are reasonable.

        Args:
            chunk_size: Proposed chunk size
            overlap_size: Proposed overlap size

        Returns:
            True if parameters are valid
        """
        if chunk_size <= 0:
            logger.error(f"Invalid chunk size: {chunk_size}. Must be positive.")
            return False

        if overlap_size < 0:
            logger.error(f"Invalid overlap size: {overlap_size}. Must be non-negative.")
            return False

        if overlap_size >= chunk_size:
            logger.error(f"Overlap size ({overlap_size}) must be less than chunk size ({chunk_size}).")
            return False

        # Additional checks could be added here
        if chunk_size > 2048:  # Arbitrary large size check
            logger.warning(f"Large chunk size detected: {chunk_size}. This may impact performance.")

        return True

    def chunk_document_content(self, document_chunks: List[DocumentChunk]) -> List[DocumentChunk]:
        """
        Process a list of document chunks and further chunk them semantically if needed.

        Args:
            document_chunks: List of DocumentChunk objects from initial processing

        Returns:
            List of DocumentChunk objects that have been semantically chunked
        """
        final_chunks = []
        total_original_chunks = len(document_chunks)

        for i, doc_chunk in enumerate(document_chunks):
            # Check if the chunk is too large and needs further chunking
            tokens = self.tokenize(doc_chunk.text_content)

            if len(tokens) > self.chunk_size:
                # This chunk is too large, so we need to split it
                logger.info(f"Splitting large chunk {i+1}/{total_original_chunks} (size: {len(tokens)} tokens)")

                # Extract the metadata from the original chunk
                chunk_metadata = {
                    **doc_chunk.metadata,
                    'original_chunk_id': doc_chunk.id,
                    'original_chunk_index': doc_chunk.chunk_index
                }

                # Create semantic sub-chunks
                sub_chunks = self.create_semantic_chunks(doc_chunk.text_content, chunk_metadata)

                # Convert to DocumentChunk format
                for j, sub_chunk in enumerate(sub_chunks):
                    new_doc_chunk = DocumentChunk(
                        id=f"{doc_chunk.id}_sub_{j}",
                        text_content=sub_chunk.text,
                        module=doc_chunk.module,
                        chapter=doc_chunk.chapter,
                        section=doc_chunk.section,
                        book_version=doc_chunk.book_version,
                        source_file_path=doc_chunk.source_file_path,
                        chunk_index=j,
                        metadata=sub_chunk.metadata,
                        vector_embedding=doc_chunk.vector_embedding  # This will be None initially
                    )
                    final_chunks.append(new_doc_chunk)
            else:
                # This chunk is fine as is
                final_chunks.append(doc_chunk)

    def validate_chunk_quality(self, chunk: Chunk) -> Tuple[bool, List[str]]:
        """
        Validate the quality of a text chunk.

        Args:
            chunk: The chunk to validate

        Returns:
            Tuple of (is_quality_ok, list_of_issues_found)
        """
        issues = []

        # Check for minimum content length
        if len(chunk.text.strip()) < 10:
            issues.append(f"Chunk {chunk.id} has very little content ({len(chunk.text)} chars)")

        # Check for too many special characters
        if chunk.text:
            special_char_ratio = len(re.findall(r'[^\w\s\n\t\r.,!?;:\-\'\"()]', chunk.text)) / len(chunk.text)
            if special_char_ratio > 0.3:  # More than 30% special characters
                issues.append(f"Chunk {chunk.id} has high ratio of special characters ({special_char_ratio:.2%})")

        # Check for presence of meaningful content
        words = re.findall(r'\b\w+\b', chunk.text.lower())
        meaningful_words = [w for w in words if len(w) > 2]  # Filter out very short words
        if len(meaningful_words) < 3:
            issues.append(f"Chunk {chunk.id} has too few meaningful words")

        return len(issues) == 0, issues

    def validate_chunks_quality(self, chunks: List[Chunk]) -> Tuple[bool, List[str], List[str]]:
        """
        Validate the quality of a list of chunks.

        Args:
            chunks: List of chunks to validate

        Returns:
            Tuple of (is_quality_ok, list_of_all_issues, list_of_chunk_ids_with_issues)
        """
        all_issues = []
        problematic_chunks = []
        overall_ok = True

        for chunk in chunks:
            is_ok, issues = self.validate_chunk_quality(chunk)
            if not is_ok:
                overall_ok = False
                problematic_chunks.append(chunk.id)
                for issue in issues:
                    all_issues.append(issue)

        return overall_ok, all_issues, problematic_chunks

    def chunk_document_content(self, document_chunks: List[DocumentChunk]) -> List[DocumentChunk]:
        """
        Process a list of document chunks and further chunk them semantically if needed.

        Args:
            document_chunks: List of DocumentChunk objects from initial processing

        Returns:
            List of DocumentChunk objects that have been semantically chunked
        """
        final_chunks = []
        total_original_chunks = len(document_chunks)

        for i, doc_chunk in enumerate(document_chunks):
            # Check if the chunk is too large and needs further chunking
            tokens = self.tokenize(doc_chunk.text_content)

            if len(tokens) > self.chunk_size:
                # This chunk is too large, so we need to split it
                logger.info(f"Splitting large chunk {i+1}/{total_original_chunks} (size: {len(tokens)} tokens)")

                # Extract the metadata from the original chunk
                chunk_metadata = {
                    **doc_chunk.metadata,
                    'original_chunk_id': doc_chunk.id,
                    'original_chunk_index': doc_chunk.chunk_index
                }

                # Create semantic sub-chunks
                sub_chunks = self.create_semantic_chunks(doc_chunk.text_content, chunk_metadata)

                # Convert to DocumentChunk format and validate quality
                for j, sub_chunk in enumerate(sub_chunks):
                    # Validate chunk quality
                    is_quality_ok, quality_issues = self.validate_chunk_quality(sub_chunk)
                    if not is_quality_ok:
                        logger.warning(f"Quality issues found in sub-chunk {sub_chunk.id}: {', '.join(quality_issues)}")
                        # Add quality issues to the chunk metadata for tracking
                        sub_chunk.metadata['quality_warnings'] = quality_issues

                    new_doc_chunk = DocumentChunk(
                        id=f"{doc_chunk.id}_sub_{j}",
                        text_content=sub_chunk.text,
                        module=doc_chunk.module,
                        chapter=doc_chunk.chapter,
                        section=doc_chunk.section,
                        book_version=doc_chunk.book_version,
                        source_file_path=doc_chunk.source_file_path,
                        chunk_index=j,
                        metadata=sub_chunk.metadata,
                        vector_embedding=doc_chunk.vector_embedding  # This will be None initially
                    )
                    final_chunks.append(new_doc_chunk)
            else:
                # This chunk is fine as is, but let's still validate its quality
                pseudo_chunk = Chunk(
                    id=doc_chunk.id,
                    text=doc_chunk.text_content,
                    index=doc_chunk.chunk_index,
                    metadata=doc_chunk.metadata
                )

                is_quality_ok, quality_issues = self.validate_chunk_quality(pseudo_chunk)
                if not is_quality_ok:
                    logger.warning(f"Quality issues found in chunk {doc_chunk.id}: {', '.join(quality_issues)}")
                    # Add quality issues to the chunk metadata for tracking
                    doc_chunk.metadata['quality_warnings'] = quality_issues

                final_chunks.append(doc_chunk)

        logger.info(f"Chunking completed: {total_original_chunks} original chunks -> {len(final_chunks)} final chunks")
        return final_chunks