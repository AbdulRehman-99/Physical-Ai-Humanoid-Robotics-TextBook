import os
from pathlib import Path
from pydantic_settings import BaseSettings
from typing import Optional


# Load environment variables from the .env file first
# Try multiple locations for .env file
env_locations = [
    Path(__file__).parent.parent.parent / ".env",  # Root directory (Text Book/)
    Path(__file__).parent.parent / ".env",         # Backend directory
    Path(__file__).parent / ".env"                 # Current directory
]

for env_path in env_locations:
    if env_path.exists():
        from dotenv import load_dotenv
        load_dotenv(dotenv_path=env_path)
        break


class Settings(BaseSettings):
    # API Keys
    cohere_api_key: str = os.getenv("COHERE_API_KEY", "")
    qdrant_api_key: Optional[str] = os.getenv("QDRANT_API_KEY")

    # Service URLs
    qdrant_url: str = os.getenv("QDRANT_URL", "")
    qdrant_host: Optional[str] = os.getenv("QDRANT_HOST", "localhost")
    qdrant_port: Optional[int] = int(os.getenv("QDRANT_PORT", "6333"))

    # Model Configuration
    embedding_model: str = os.getenv("EMBEDDING_MODEL", "embed-multilingual-v2.0")  # Cohere model

    # OpenRouter Configuration
    openrouter_api_key: str = os.getenv("OPENROUTER_API_KEY", "")
    openrouter_base_url: str = os.getenv("OPENROUTER_BASE_URL", "https://openrouter.ai/api/v1")
    openrouter_model: str = os.getenv("OPENROUTER_MODEL", "qwen-2.5-72b-instruct")

    # Vector Database Configuration
    collection_name: str = os.getenv("COLLECTION_NAME", "book_content")
    top_k: int = int(os.getenv("TOP_K", "3"))

    # Application Configuration
    host: str = os.getenv("HOST", "0.0.0.0")
    port: int = int(os.getenv("PORT", "8000"))
    debug: bool = os.getenv("DEBUG", "False").lower() == "true"

    class Config:
        env_file_encoding = "utf-8"


settings = Settings()