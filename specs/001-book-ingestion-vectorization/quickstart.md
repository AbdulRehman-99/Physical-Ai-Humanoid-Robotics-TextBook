# Quickstart Guide: Book Ingestion System

## Prerequisites

- Python 3.11 or higher
- UV package manager
- Access to Cohere API
- Access to Qdrant vector database

## Setup

1. **Create the Backend directory:**
   ```bash
   mkdir Backend
   cd Backend
   ```

2. **Initialize the project with UV:**
   ```bash
   uv init
   ```

3. **Create the .env file with required API keys:**
   ```bash
   touch .env
   ```

4. **Add environment variables to .env:**
   ```
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   QDRANT_URL=your_qdrant_cluster_url_here
   ```

5. **Install required dependencies:**
   ```bash
   uv add cohere qdrant-client python-dotenv markdown
   ```

## Usage

1. **Place your main ingestion script as `ingest_book.py`**

2. **Run the ingestion process:**
   ```bash
   python ingest_book.py
   ```

## Expected Structure

After setup, your Backend directory should look like:
```
Backend/
├── pyproject.toml
├── ingest_book.py
├── .env
├── .gitignore
└── src/
    └── (your source modules)
```

## Verification

After running the ingestion script:
- Check that ≥95% of markdown files from `Frontend/docs` have been processed successfully
- Verify that embeddings have been stored in Qdrant with proper metadata
- Confirm that metadata (module, chapter, section, book_version) is preserved
- Test RAG chatbot integration by performing sample semantic searches
- Verify that search queries return results within 2 seconds