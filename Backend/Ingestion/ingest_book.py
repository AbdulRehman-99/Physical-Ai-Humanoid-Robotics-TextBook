#!/usr/bin/env python3
"""Main script to orchestrate the book ingestion pipeline."""

import argparse
import sys
import os
from pathlib import Path
import logging
from datetime import datetime
from typing import List

# Add the src directory to the path so we can import modules
sys.path.insert(0, str(Path(__file__).parent / 'src'))

from src.ingestion.document_processor import DocumentProcessor
from src.ingestion.vectorizer import Vectorizer, create_embedding_pipeline
from src.config.settings import settings
from src.ingestion.logging_config import setup_logging
from src.ingestion.models import ProcessingResult


def parse_arguments():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Book Ingestion Pipeline - Process markdown files and store embeddings in Qdrant"
    )

    parser.add_argument(
        '--source-path',
        type=str,
        default='Frontend/docs',
        help='Path to the source markdown files (default: Frontend/docs)'
    )

    parser.add_argument(
        '--target-collection',
        type=str,
        default=None,
        help='Target Qdrant collection name (uses default from settings if not specified)'
    )

    parser.add_argument(
        '--validate-quality',
        action='store_true',
        default=True,
        help='Validate embedding quality before storing (default: True)'
    )

    parser.add_argument(
        '--chunk-size',
        type=int,
        default=None,
        help='Chunk size for semantic chunking (uses default from settings if not specified)'
    )

    parser.add_argument(
        '--chunk-overlap',
        type=int,
        default=None,
        help='Overlap size between chunks (uses default from settings if not specified)'
    )

    parser.add_argument(
        '--log-level',
        type=str,
        default='INFO',
        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'],
        help='Logging level (default: INFO)'
    )

    return parser.parse_args()


def main():
    """Main function to run the complete ingestion pipeline."""
    start_time = datetime.now()

    # Parse command-line arguments
    args = parse_arguments()

    # Setup logging with the specified level
    setup_logging(level=args.log_level)
    logger = logging.getLogger(__name__)

    logger.info("Starting book ingestion pipeline...")
    logger.info(f"Source path: {args.source_path}")
    logger.info(f"Target collection: {args.target_collection or settings.qdrant_collection_name}")

    try:
        # Update settings if command-line parameters are provided
        if args.chunk_size:
            settings.chunk_size = args.chunk_size
        if args.chunk_overlap:
            settings.chunk_overlap = args.chunk_overlap
        if args.log_level:
            settings.log_level = args.log_level

        logger.info(f"Using chunk size: {settings.chunk_size}, overlap: {settings.chunk_overlap}")

        # Initialize components
        logger.info("Initializing document processor...")
        processor = DocumentProcessor()

        logger.info("Initializing vectorizer...")
        vectorizer = Vectorizer()

        # Process documents
        logger.info("Starting document processing...")
        processing_results = processor.process_documents(args.source_path)

        # Calculate and report summary statistics
        total_docs = len(processing_results)
        successful_docs = len([r for r in processing_results if r.status == "success"])
        failed_docs = len([r for r in processing_results if r.status == "failed"])
        total_chunks = sum(r.chunks_created for r in processing_results if r.status == "success")

        logger.info(f"Document processing completed: {successful_docs}/{total_docs} documents processed successfully")

        # Process embeddings only if there were successful documents
        if successful_docs > 0:
            logger.info("Starting embedding generation and storage...")

            # We need to process the documents again to get the actual chunks
            # This is inefficient but necessary since process_documents doesn't return the chunks
            markdown_files = processor.scan_documents(args.source_path)
            all_chunks = []

            for file_path in markdown_files:
                try:
                    chunks = processor.process_single_document(file_path)
                    all_chunks.extend(chunks)
                    logger.info(f"Retrieved {len(chunks)} chunks from {file_path}")
                except Exception as e:
                    logger.error(f"Failed to retrieve chunks from {file_path}: {e}")

            if all_chunks:
                logger.info(f"Processing {len(all_chunks)} total chunks for embedding...")

                # Process and store all chunks using the vectorizer
                success = vectorizer.process_and_store_chunks(all_chunks)

                if success:
                    logger.info("Embedding generation and storage completed successfully!")
                else:
                    logger.error("Embedding generation or storage failed!")
                    return 1
            else:
                logger.warning("No chunks were retrieved for embedding!")
        else:
            logger.warning("No documents were processed successfully, skipping embedding step.")

        end_time = datetime.now()
        duration = end_time - start_time

        logger.info("=" * 60)
        logger.info("INGESTION PIPELINE SUMMARY")
        logger.info("=" * 60)
        logger.info(f"Start time: {start_time.strftime('%Y-%m-%d %H:%M:%S')}")
        logger.info(f"End time: {end_time.strftime('%Y-%m-%d %H:%M:%S')}")
        logger.info(f"Duration: {duration}")
        logger.info(f"Total documents processed: {total_docs}")
        logger.info(f"Successful: {successful_docs}")
        logger.info(f"Failed: {failed_docs}")
        logger.info(f"Total chunks created: {total_chunks}")
        logger.info(f"Target collection: {args.target_collection or settings.qdrant_collection_name}")
        logger.info("=" * 60)

        # Return success if we had successful document processing
        if successful_docs > 0:
            logger.info("Pipeline completed successfully!")
            return 0
        else:
            logger.error("Pipeline completed but no documents were processed successfully!")
            return 1

    except KeyboardInterrupt:
        logger.error("Pipeline interrupted by user!")
        return 130  # Standard exit code for Ctrl+C
    except Exception as e:
        logger.error(f"Pipeline failed with error: {e}")
        logger.exception("Full traceback:")
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)