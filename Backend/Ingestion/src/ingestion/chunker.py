import re
from typing import List, Tuple, Optional, Set
from dataclasses import dataclass
from src.ingestion.models import DocumentChunk
from src.config.settings import settings
import logging

logger = logging.getLogger(__name__)

MARKDOWN_HEADING = re.compile(r'^(#{1,6})\s+(.+)$', re.MULTILINE)


@dataclass
class Chunk:
    id: str
    text: str
    index: int
    metadata: dict


class Chunker:
    """Recursive markdown-aware chunker with priority-based splitting."""

    def __init__(self, chunk_size: int = None, overlap_size: int = None):
        self.chunk_size = chunk_size or settings.chunk_size
        self.overlap_size = overlap_size or settings.chunk_overlap

        if self.overlap_size >= self.chunk_size:
            raise ValueError(
                f"Overlap size ({self.overlap_size}) must be less than chunk size ({self.chunk_size})"
            )

    def _estimate_tokens(self, text: str) -> int:
        return len(text.split())

    def _split_by_headings(self, text: str) -> List[str]:
        matches = list(MARKDOWN_HEADING.finditer(text))
        if not matches:
            return [text]

        sections = []
        for i, match in enumerate(matches):
            start = match.start()
            if i > 0:
                sections.append(text[matches[i - 1].start():start].strip())
            if i == len(matches) - 1:
                sections.append(text[start:].strip())
        return [s for s in sections if s]

    def _split_by_paragraphs(self, text: str) -> List[str]:
        paragraphs = re.split(r'\n\s*\n', text)
        return [p.strip() for p in paragraphs if p.strip()]

    def _split_by_sentences(self, text: str) -> List[str]:
        sentences = re.split(r'(?<=[.!?])\s+', text)
        return [s.strip() for s in sentences if s.strip()]

    def _recursive_split(self, text: str, depth: int = 0) -> List[str]:
        if self._estimate_tokens(text) <= self.chunk_size:
            return [text]

        if depth == 0:
            parts = self._split_by_headings(text)
        elif depth == 1:
            parts = self._split_by_paragraphs(text)
        elif depth == 2:
            parts = self._split_by_sentences(text)
        else:
            return self._split_by_token_window(text)

        if len(parts) <= 1:
            return self._recursive_split(text, depth + 1)

        result = []
        buffer = ""
        for part in parts:
            candidate = (buffer + "\n\n" + part).strip() if buffer else part
            if self._estimate_tokens(candidate) <= self.chunk_size:
                buffer = candidate
            else:
                if buffer:
                    result.append(buffer)
                if self._estimate_tokens(part) <= self.chunk_size:
                    buffer = part
                else:
                    result.extend(self._recursive_split(part, depth + 1))
                    buffer = ""
        if buffer:
            result.append(buffer)
        return result

    def _split_by_token_window(self, text: str) -> List[str]:
        tokens = text.split()
        chunks = []
        start = 0
        while start < len(tokens):
            end = min(start + self.chunk_size, len(tokens))
            chunk_text = " ".join(tokens[start:end])
            chunks.append(chunk_text)
            next_start = max(end - self.overlap_size, start + 1)
            if next_start <= start:
                next_start = end
            start = next_start
        return chunks

    def create_chunks(self, text: str, doc_metadata: dict = None) -> List[Chunk]:
        if not text.strip():
            return []

        if self._estimate_tokens(text) <= self.chunk_size:
            return [Chunk(
                id="chunk_0",
                text=text.strip(),
                index=0,
                metadata=doc_metadata or {},
            )]

        parts = self._recursive_split(text)
        chunks = []
        for i, part in enumerate(parts):
            chunk = Chunk(
                id=f"chunk_{i}",
                text=part,
                index=i,
                metadata=doc_metadata or {},
            )
            chunks.append(chunk)

        logger.info(f"Created {len(chunks)} chunks from text of length {self._estimate_tokens(text)} tokens")
        return chunks

    def validate_chunk_quality(self, chunk: Chunk) -> Tuple[bool, List[str]]:
        issues = []

        if len(chunk.text.strip()) < 10:
            issues.append(f"Chunk {chunk.id} has very little content ({len(chunk.text)} chars)")

        if chunk.text:
            special_char_ratio = len(re.findall(r'[^\w\s\n\t\r.,!?;:\-\'\"()]', chunk.text)) / len(chunk.text)
            if special_char_ratio > 0.3:
                issues.append(f"Chunk {chunk.id} has high ratio of special characters ({special_char_ratio:.2%})")

        words = re.findall(r'\b\w+\b', chunk.text.lower())
        meaningful_words = [w for w in words if len(w) > 2]
        if len(meaningful_words) < 3:
            issues.append(f"Chunk {chunk.id} has too few meaningful words")

        return len(issues) == 0, issues

    def validate_chunks_quality(self, chunks: List[Chunk]) -> Tuple[bool, List[str], List[str]]:
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
        final_chunks = []
        total_original_chunks = len(document_chunks)

        for i, doc_chunk in enumerate(document_chunks):
            token_count = self._estimate_tokens(doc_chunk.text_content)

            if token_count > self.chunk_size:
                logger.info(f"Splitting large chunk {i+1}/{total_original_chunks} (size: {token_count} tokens)")

                chunk_metadata = {
                    **(doc_chunk.metadata or {}),
                    "original_chunk_id": doc_chunk.id,
                    "original_chunk_index": doc_chunk.chunk_index,
                }

                sub_chunks = self.create_chunks(doc_chunk.text_content, chunk_metadata)

                for j, sub_chunk in enumerate(sub_chunks):
                    is_quality_ok, quality_issues = self.validate_chunk_quality(sub_chunk)
                    if not is_quality_ok:
                        logger.warning(f"Quality issues found in sub-chunk {sub_chunk.id}: {', '.join(quality_issues)}")
                        sub_chunk.metadata["quality_warnings"] = quality_issues

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
                        vector_embedding=doc_chunk.vector_embedding,
                    )
                    final_chunks.append(new_doc_chunk)
            else:
                pseudo_chunk = Chunk(
                    id=doc_chunk.id,
                    text=doc_chunk.text_content,
                    index=doc_chunk.chunk_index,
                    metadata=doc_chunk.metadata,
                )

                is_quality_ok, quality_issues = self.validate_chunk_quality(pseudo_chunk)
                if not is_quality_ok:
                    logger.warning(f"Quality issues found in chunk {doc_chunk.id}: {', '.join(quality_issues)}")
                    if doc_chunk.metadata is None:
                        doc_chunk.metadata = {}
                    doc_chunk.metadata["quality_warnings"] = quality_issues

                final_chunks.append(doc_chunk)

        logger.info(f"Chunking completed: {total_original_chunks} original chunks -> {len(final_chunks)} final chunks")
        return final_chunks
