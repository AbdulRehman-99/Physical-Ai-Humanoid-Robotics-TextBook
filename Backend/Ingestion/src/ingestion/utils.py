import os
import logging
from pathlib import Path
from typing import List, Optional
import hashlib

def get_all_markdown_files(base_path: str) -> List[Path]:
    """
    Recursively find all markdown files in the given base path.

    Args:
        base_path: The directory path to search for markdown files

    Returns:
        List of Path objects for all .md files found
    """
    base_path = Path(base_path)
    if not base_path.exists():
        logging.warning(f"Base path does not exist: {base_path}")
        return []

    markdown_files = []
    for file_path in base_path.rglob("*.md"):
        if file_path.is_file():
            markdown_files.append(file_path)

    return markdown_files

def calculate_file_hash(file_path: Path) -> str:
    """
    Calculate MD5 hash of a file.

    Args:
        file_path: Path to the file

    Returns:
        MD5 hash as hexadecimal string
    """
    hash_md5 = hashlib.md5()
    with open(file_path, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            hash_md5.update(chunk)
    return hash_md5.hexdigest()

def get_file_size(file_path: Path) -> int:
    """
    Get the size of a file in bytes.

    Args:
        file_path: Path to the file

    Returns:
        Size of the file in bytes
    """
    return file_path.stat().st_size

def ensure_directory_exists(directory_path: str) -> bool:
    """
    Ensure that a directory exists, creating it if necessary.

    Args:
        directory_path: Path to the directory

    Returns:
        True if directory exists or was created successfully, False otherwise
    """
    try:
        Path(directory_path).mkdir(parents=True, exist_ok=True)
        return True
    except Exception as e:
        logging.error(f"Failed to create directory {directory_path}: {e}")
        return False

def sanitize_filename(filename: str) -> str:
    """
    Sanitize a filename by removing or replacing invalid characters.

    Args:
        filename: Original filename

    Returns:
        Sanitized filename
    """
    # Replace invalid characters for most filesystems
    invalid_chars = '<>:"/\\|?*'
    sanitized = filename
    for char in invalid_chars:
        sanitized = sanitized.replace(char, '_')
    return sanitized