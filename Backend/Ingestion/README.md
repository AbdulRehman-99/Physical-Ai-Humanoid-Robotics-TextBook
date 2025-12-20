# Book Ingestion System

This project processes markdown files from the Frontend/docs directory, converts them to vector embeddings using Cohere, and stores them in Qdrant for semantic search capabilities.

## Setup

1. Install dependencies: `pip install -r requirements.txt` (or use uv to install: `uv pip install cohere qdrant-client python-dotenv markdown pytest`)
2. Create `.env` file with your API keys:
   ```
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   QDRANT_URL=your_qdrant_cluster_url_here
   ```
3. Run the ingestion: `python ingest_book.py`

## Features

- Processes all .md files in Frontend/docs (glossary, modules, chapters)
- Cleans and normalizes text content
- Preserves document metadata (module, chapter, section, book_version)
- Performs semantic chunking with configurable overlap
- Generates embeddings using Cohere API
- Stores vectors with metadata in Qdrant vector database
- Enables semantic search for RAG chatbot integration