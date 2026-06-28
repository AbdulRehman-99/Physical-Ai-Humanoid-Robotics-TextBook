import asyncio
import io
import logging
import time
from typing import Optional

from .config import TTSConfig

try:
    import edge_tts
except ImportError:
    edge_tts = None

logger = logging.getLogger(__name__)


class TTSError(Exception):
    pass


class TTSTimeoutError(TTSError):
    pass


class TTSConfigError(TTSError):
    pass


class TTSEngine:
    def __init__(self, config: Optional[TTSConfig] = None):
        self.config = config or TTSConfig()

    async def synthesize(
        self,
        text: str,
        voice: Optional[str] = None,
        rate: Optional[str] = None,
    ) -> bytes:
        if edge_tts is None:
            raise TTSConfigError("edge-tts is not installed. Run: pip install edge-tts")

        effective_voice = voice or self.config.DEFAULT_VOICE
        effective_rate = rate or self.config.DEFAULT_RATE

        if not text or not text.strip():
            raise TTSError("text must not be empty")

        if len(text) > self.config.MAX_TEXT_LENGTH:
            raise TTSError(
                f"text exceeds maximum length of {self.config.MAX_TEXT_LENGTH} characters"
            )

        logger.info(
            "Synthesizing %d chars with voice=%s rate=%s",
            len(text),
            effective_voice,
            effective_rate,
        )

        start = time.perf_counter()

        try:
            communicate = edge_tts.Communicate(text, effective_voice, rate=effective_rate)
            buffer = io.BytesIO()

            async for chunk in communicate.stream():
                if chunk["type"] == "audio":
                    buffer.write(chunk["data"])

            audio_bytes = buffer.getvalue()
            elapsed = time.perf_counter() - start
            logger.info(
                "Synthesis complete: %d bytes in %.2fs",
                len(audio_bytes),
                elapsed,
            )
            return audio_bytes

        except asyncio.TimeoutError:
            raise TTSTimeoutError(
                f"TTS synthesis timed out after {self.config.TIMEOUT_SECONDS}s"
            )
        except Exception as e:
            logger.error("TTS synthesis failed: %s", str(e))
            raise TTSError(f"TTS synthesis failed: {e}") from e
