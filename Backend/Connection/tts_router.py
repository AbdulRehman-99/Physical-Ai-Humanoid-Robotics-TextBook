import asyncio
import io
import logging
import time

from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse

from .models import TTSRequest
from .config import TTS_ENABLED, TTS_DEFAULT_VOICE, TTS_TIMEOUT_SECONDS, TTS_MAX_TEXT_LENGTH
from services.tts import TTSEngine, TTSError, TTSTimeoutError

logger = logging.getLogger(__name__)

tts_router = APIRouter(prefix="/tts", tags=["tts"])


@tts_router.post("")
async def synthesize_speech(request: TTSRequest):
    if not TTS_ENABLED:
        raise HTTPException(status_code=503, detail="TTS is currently disabled")

    if not request.text or not request.text.strip():
        raise HTTPException(status_code=400, detail="text must not be empty")

    if len(request.text) > TTS_MAX_TEXT_LENGTH:
        raise HTTPException(
            status_code=400,
            detail=f"text exceeds maximum length of {TTS_MAX_TEXT_LENGTH} characters",
        )

    try:
        engine = TTSEngine()
        voice = request.voice or TTS_DEFAULT_VOICE

        audio_bytes = await asyncio.wait_for(
            engine.synthesize(request.text, voice=voice, rate=request.rate),
            timeout=TTS_TIMEOUT_SECONDS,
        )

        duration_ms = int(len(audio_bytes) / 16)

        logger.info(
            "TTS success: %d chars, %d bytes, voice=%s",
            len(request.text),
            len(audio_bytes),
            voice,
        )

        return StreamingResponse(
            io.BytesIO(audio_bytes),
            media_type="audio/mpeg",
            headers={
                "X-TTS-Voice": voice,
                "X-TTS-Duration-Ms": str(duration_ms),
                "Content-Length": str(len(audio_bytes)),
            },
        )

    except asyncio.TimeoutError:
        logger.error("TTS request timed out after %ds", TTS_TIMEOUT_SECONDS)
        raise HTTPException(
            status_code=504,
            detail=f"TTS synthesis timed out after {TTS_TIMEOUT_SECONDS}s",
        )
    except TTSTimeoutError as e:
        raise HTTPException(status_code=504, detail=str(e))
    except TTSError as e:
        logger.error("TTS synthesis error: %s", str(e))
        raise HTTPException(status_code=500, detail=str(e))
    except Exception as e:
        logger.error("Unexpected TTS error: %s", str(e))
        raise HTTPException(status_code=500, detail="TTS synthesis failed")
