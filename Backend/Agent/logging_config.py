import logging
import sys
from logging.handlers import RotatingFileHandler
from settings import settings


def setup_logging():
    """
    Set up comprehensive logging configuration
    """
    # Create a custom logger
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG if settings.debug else logging.INFO)

    # Remove existing handlers to avoid duplicates
    for handler in logger.handlers[:]:
        logger.removeHandler(handler)

    # Create formatters and add it to handlers
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # Create console handler for stdout
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.DEBUG if settings.debug else logging.INFO)
    console_handler.setFormatter(formatter)

    # Create file handler for rotating log files
    file_handler = RotatingFileHandler(
        'rag_agent.log',
        maxBytes=10*1024*1024,  # 10MB
        backupCount=5
    )
    file_handler.setLevel(logging.INFO)
    file_handler.setFormatter(formatter)

    # Add handlers to the logger
    logger.addHandler(console_handler)
    logger.addHandler(file_handler)

    # Suppress overly verbose logs from external libraries
    logging.getLogger("uvicorn").setLevel(logging.WARNING)
    logging.getLogger("urllib3").setLevel(logging.WARNING)
    logging.getLogger("qdrant_client").setLevel(logging.WARNING)
    logging.getLogger("httpx").setLevel(logging.WARNING)

    return logger


def add_error_logging_middleware(app):
    """
    Add error logging middleware to the FastAPI app
    """
    @app.middleware("http")
    async def log_errors(request, call_next):
        try:
            response = await call_next(request)
            return response
        except Exception as e:
            # Log the error with context
            logging.error(f"Error processing request: {request.method} {request.url}", exc_info=True)
            raise


# Initialize logging when module is imported
setup_logging()