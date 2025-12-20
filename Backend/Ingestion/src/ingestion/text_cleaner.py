"""Module for cleaning and normalizing text content."""

import re
import html
from typing import List, Tuple
from bs4 import BeautifulSoup  # For HTML parsing
import unicodedata
import logging

logger = logging.getLogger(__name__)

class TextCleaner:
    """Class to handle text cleaning and normalization operations."""

    def __init__(self):
        """Initialize the text cleaner."""
        pass

    def remove_markdown_formatting(self, text: str) -> str:
        """
        Remove markdown formatting while preserving content.

        Args:
            text: Input text with markdown formatting

        Returns:
            Text with markdown formatting removed
        """
        # Remove markdown headers (### Header -> Header)
        text = re.sub(r'^#+\s*', '', text, flags=re.MULTILINE)

        # Remove bold and italic formatting
        text = re.sub(r'\*{1,2}(.*?)\*{1,2}', r'\1', text)  # *text* and **text**
        text = re.sub(r'_{1,2}(.*?)_{1,2}', r'\1', text)   # _text_ and __text__

        # Remove code blocks (inline and block)
        text = re.sub(r'`{1,3}[^`]*?`{1,3}', '', text)

        # Remove links [text](url) -> text
        text = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', text)

        # Remove images ![alt](url) -> alt
        text = re.sub(r'!\[([^\]]*)\]\([^)]+\)', r'\1', text)

        # Remove horizontal rules
        text = re.sub(r'^\s*[-*_]{3,}\s*$', '', text, flags=re.MULTILINE)

        # Remove blockquotes
        text = re.sub(r'^\s*>\s?', '', text, flags=re.MULTILINE)

        # Remove list markers
        text = re.sub(r'^\s*[\*\-\+]\s+', '', text, flags=re.MULTILINE)
        text = re.sub(r'^\s*\d+\.\s+', '', text, flags=re.MULTILINE)

        return text.strip()

    def normalize_whitespace(self, text: str) -> str:
        """
        Normalize whitespace and line breaks in text.

        Args:
            text: Input text

        Returns:
            Text with normalized whitespace
        """
        # Normalize different types of whitespace to regular spaces
        text = re.sub(r'[ \t]+', ' ', text)  # Multiple spaces/tabs to single space

        # Normalize line breaks
        text = re.sub(r'\r\n', '\n', text)  # Windows line breaks to Unix
        text = re.sub(r'\r', '\n', text)    # Old Mac line breaks to Unix

        # Normalize multiple line breaks to double line breaks (preserve paragraph structure)
        text = re.sub(r'\n{3,}', '\n\n', text)

        # Remove leading/trailing whitespace from each line
        lines = [line.strip() for line in text.split('\n')]
        text = '\n'.join(lines)

        # Remove extra leading/trailing whitespace from entire text
        text = text.strip()

        return text

    def handle_special_characters(self, text: str) -> str:
        """
        Handle special characters and encoding issues.

        Args:
            text: Input text

        Returns:
            Text with special characters handled
        """
        # Unescape HTML entities
        text = html.unescape(text)

        # Remove or replace control characters (except \n, \t, \r)
        text = ''.join(char if ord(char) >= 32 or char in '\n\t\r' else ' ' for char in text)

        # Normalize unicode characters (e.g., convert accented characters to their base form)
        text = unicodedata.normalize('NFKD', text)

        # Replace various quotation marks with standard ones
        text = re.sub(r'[`\'`´]', "'", text)  # Various single quotes to standard
        text = re.sub(r'["""]', '"', text)   # Various double quotes to standard

        # Replace various dashes with standard hyphen
        text = re.sub(r'[–—−]', '-', text)  # En-dash, em-dash, minus sign to hyphen

        return text

    def clean_html_entities(self, text: str) -> str:
        """
        Clean HTML entities and special symbols.

        Args:
            text: Input text

        Returns:
            Text with HTML entities cleaned
        """
        # Unescape HTML entities
        text = html.unescape(text)

        # Use BeautifulSoup to clean any remaining HTML tags
        soup = BeautifulSoup(text, 'html.parser')
        text = soup.get_text()

        # Remove extra whitespace that might be introduced
        text = re.sub(r'\s+', ' ', text).strip()

        return text

    def normalize_text_case(self, text: str) -> str:
        """
        Normalize text case where appropriate.

        Args:
            text: Input text

        Returns:
            Text with appropriate case normalization
        """
        # For now, we'll just ensure consistent capitalization
        # This can be expanded based on specific requirements
        return text

    def handle_encoding_formats(self, text: str) -> str:
        """
        Handle different encoding formats.

        Args:
            text: Input text

        Returns:
            Text with consistent encoding
        """
        # The actual encoding handling would happen at the file reading level
        # Here we just ensure the text is in a consistent format
        try:
            # Ensure text is properly decoded
            if isinstance(text, bytes):
                text = text.decode('utf-8', errors='ignore')
        except:
            # If there are any issues, return the original text
            pass

        return text

    def quality_check(self, text: str) -> Tuple[bool, List[str]]:
        """
        Perform quality checks on cleaned text.

        Args:
            text: Text to check

        Returns:
            Tuple of (is_quality_ok, list_of_issues_found)
        """
        issues = []

        # Check for excessive special characters
        special_char_ratio = len(re.findall(r'[^\w\s\n\t\r.,!?;:\-]', text)) / len(text) if text else 0
        if special_char_ratio > 0.3:  # More than 30% special characters
            issues.append("High ratio of special characters")

        # Check for null bytes
        if '\x00' in text:
            issues.append("Contains null bytes")

        # Check for very long lines (could indicate formatting issues)
        lines = text.split('\n')
        if any(len(line) > 1000 for line in lines):
            issues.append("Contains very long lines (>1000 chars)")

        return len(issues) == 0, issues

    def clean_text(self, text: str) -> str:
        """
        Apply all cleaning operations to text in the proper order.

        Args:
            text: Input text to clean

        Returns:
            Cleaned and normalized text
        """
        try:
            # Apply cleaning operations in sequence
            text = self.remove_markdown_formatting(text)
            text = self.clean_html_entities(text)
            text = self.handle_special_characters(text)
            text = self.normalize_whitespace(text)
            text = self.handle_encoding_formats(text)

            # Perform quality check
            is_ok, issues = self.quality_check(text)
            if not is_ok:
                logger.warning(f"Quality issues found in text: {', '.join(issues)}")

            return text
        except Exception as e:
            logger.error(f"Error during text cleaning: {e}")
            # Return original text if cleaning fails
            return text