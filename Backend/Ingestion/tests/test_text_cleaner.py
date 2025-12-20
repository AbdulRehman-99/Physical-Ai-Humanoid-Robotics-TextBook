"""Unit tests for the text cleaner module."""

import unittest
from src.ingestion.text_cleaner import TextCleaner


class TestTextCleaner(unittest.TestCase):
    """Test cases for TextCleaner class."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.cleaner = TextCleaner()

    def test_remove_markdown_formatting(self):
        """Test removing various markdown formatting."""
        markdown_text = """
# Header 1
## Header 2
This is **bold** and *italic* text.
This is __bold__ and _italic_ text too.
[Link text](http://example.com)
![Alt text](image.jpg)
- List item 1
- List item 2
1. Numbered item 1
2. Numbered item 2
`inline code`
"""
        expected = """Header 1
Header 2
This is bold and italic text.
This is bold and italic text too.
Link text
Alt text
List item 1
List item 2
Numbered item 1
Numbered item 2
"""
        result = self.cleaner.remove_markdown_formatting(markdown_text)
        self.assertEqual(result.strip(), expected.strip())

    def test_normalize_whitespace(self):
        """Test normalizing whitespace and line breaks."""
        text_with_whitespace = "  Multiple   spaces\tand\tsome\ttabs  \n\n\nMultiple\n\nline\nbreaks  "
        expected = "Multiple spaces and some tabs\n\nMultiple\n\nline breaks"
        result = self.cleaner.normalize_whitespace(text_with_whitespace)
        self.assertEqual(result, expected)

    def test_handle_special_characters(self):
        """Test handling special characters."""
        text_with_special = "Text with &amp; HTML entity, quotes 'and' \"double\", and dash – en-dash—em-dash"
        result = self.cleaner.handle_special_characters(text_with_special)
        # The function should unescape HTML entities and normalize quotes/dashes
        self.assertIn("Text with & HTML entity", result)
        self.assertIn("'and'", result)  # Single quotes normalized
        self.assertIn('"double"', result)  # Double quotes normalized
        self.assertIn("dash - en-dash-em-dash", result)  # Dashes normalized to hyphens

    def test_clean_html_entities(self):
        """Test cleaning HTML entities."""
        html_text = "<p>Paragraph with <b>bold</b> and <i>italic</i> tags.</p>"
        result = self.cleaner.clean_html_entities(html_text)
        # Should remove HTML tags but preserve content
        self.assertIn("Paragraph with bold and italic tags.", result)
        self.assertNotIn("<p>", result)
        self.assertNotIn("<b>", result)

    def test_normalize_text_case(self):
        """Test text case normalization."""
        text = "This is a test."
        result = self.cleaner.normalize_text_case(text)
        # For now, this function just returns the text as-is
        self.assertEqual(result, text)

    def test_quality_check_good_text(self):
        """Test quality check with good text."""
        good_text = "This is a normal text with regular characters."
        is_ok, issues = self.cleaner.quality_check(good_text)
        self.assertTrue(is_ok)
        self.assertEqual(len(issues), 0)

    def test_quality_check_poor_text(self):
        """Test quality check with poor quality text."""
        poor_text = "This text has too many $$$ special @@@ characters !!!"
        is_ok, issues = self.cleaner.quality_check(poor_text)
        self.assertFalse(is_ok)
        # Should identify high ratio of special characters
        self.assertTrue(any("special characters" in issue for issue in issues))

    def test_clean_text_complete(self):
        """Test the complete text cleaning function."""
        markdown_text = """
# Header
This is **bold** text with &amp; HTML entity.
Multiple    spaces and\tsome\ttabs.
"""
        result = self.cleaner.clean_text(markdown_text)
        # Should have cleaned formatting, normalized spaces, and unescaped HTML
        self.assertNotIn("# Header", result)
        self.assertNotIn("**bold**", result)
        self.assertIn("Header", result)
        self.assertIn("bold", result)
        self.assertIn("Multiple spaces", result)  # Multiple spaces normalized
        self.assertIn("HTML entity", result)  # HTML entity unescaped


if __name__ == '__main__':
    unittest.main()