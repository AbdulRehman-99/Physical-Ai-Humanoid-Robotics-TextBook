# Configuration for timeout and retry mechanisms
TIMEOUT_SECONDS = 30  # Configurable timeout value
RETRY_ATTEMPTS = 3    # Number of retry attempts
EXPONENTIAL_BACKOFF_BASE = 2  # For exponential backoff calculation

# Rate limiting configuration
RATE_LIMIT_MAX_REQUESTS = 100  # Max requests per minute per IP
RATE_LIMIT_WINDOW = 60  # Time window in seconds

# Qdrant configuration
QDRANT_TOP_K = 3  # Default number of chunks to retrieve

# Agent configuration
AGENT_TIMEOUT = 30  # Timeout for agent calls
AGENT_RETRY_ATTEMPTS = 3  # Retry attempts for agent calls

# Session configuration
SESSION_MEMORY_TURNS = 5  # Number of conversation turns to keep in memory
SESSION_TTL_SECONDS = 1800  # Session idle timeout (30 min)

# TTS configuration
TTS_ENABLED = True
TTS_DEFAULT_VOICE = "en-US-AvaNeural"
TTS_TIMEOUT_SECONDS = 30
TTS_MAX_TEXT_LENGTH = 5000