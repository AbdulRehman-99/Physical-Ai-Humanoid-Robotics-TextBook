import os
from dataclasses import dataclass, field


@dataclass
class TTSConfig:
    DEFAULT_VOICE: str = os.getenv("TTS_DEFAULT_VOICE", "en-US-AvaNeural")
    DEFAULT_RATE: str = os.getenv("TTS_DEFAULT_RATE", "+0%")
    MAX_TEXT_LENGTH: int = int(os.getenv("TTS_MAX_TEXT_LENGTH", "5000"))
    TIMEOUT_SECONDS: int = int(os.getenv("TTS_TIMEOUT_SECONDS", "30"))
    STREAM_CHUNK_SIZE: int = 8192
