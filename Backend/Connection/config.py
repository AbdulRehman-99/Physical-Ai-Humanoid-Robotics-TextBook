# Configuration for timeout and retry mechanisms
TIMEOUT_SECONDS = 30  # Configurable timeout value
RETRY_ATTEMPTS = 3    # Number of retry attempts
EXPONENTIAL_BACKOFF_BASE = 2  # For exponential backoff calculation

# Rate limiting configuration
RATE_LIMIT_MAX_REQUESTS = 100  # Max requests per minute per IP
RATE_LIMIT_WINDOW = 60  # Time window in seconds

# Qdrant configuration
QDRANT_TOP_K = 5  # Default number of chunks to retrieve

# Agent configuration
AGENT_TIMEOUT = 30  # Timeout for agent calls
AGENT_RETRY_ATTEMPTS = 3  # Retry attempts for agent calls