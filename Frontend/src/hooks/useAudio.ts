import { useState, useEffect, useRef, useCallback } from 'react';

type AudioState = 'idle' | 'loading' | 'playing' | 'paused' | 'error';

interface UseAudioReturn {
  state: AudioState;
  error: string | null;
  play: (text: string, messageId: string) => Promise<void>;
  stop: () => void;
  restart: () => Promise<void>;
}

const isProduction = typeof window !== 'undefined' &&
  window.location.hostname !== 'localhost' &&
  !window.location.hostname.startsWith('127.');

const BACKEND_URLS = isProduction
  ? ['https://abdul-rehman-99-textbook.hf.space']
  : ['http://localhost:8000', 'http://localhost:8001', 'http://localhost:8002'];

let currentAudio: HTMLAudioElement | null = null;
let currentKey: string | null = null;

function stopCurrentAudio(): void {
  if (currentAudio) {
    currentAudio.pause();
    currentAudio.onended = null;
    currentAudio.onerror = null;
    currentAudio = null;
  }
  currentKey = null;
}

export function useAudio(): UseAudioReturn {
  const [state, setState] = useState<AudioState>('idle');
  const [error, setError] = useState<string | null>(null);
  const errorTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const mountedRef = useRef(true);

  useEffect(() => {
    mountedRef.current = true;
    return () => {
      mountedRef.current = false;
      if (errorTimerRef.current) {
        clearTimeout(errorTimerRef.current);
      }
    };
  }, []);

  const resetError = useCallback(() => {
    if (!mountedRef.current) return;
    setState('idle');
    setError(null);
  }, []);

  const play = useCallback(async (text: string, messageId: string) => {
    if (!text || !text.trim()) return;

    if (currentKey === messageId && currentAudio && !currentAudio.paused) {
      currentAudio.pause();
      setState('paused');
      return;
    }

    if (currentKey === messageId && currentAudio && currentAudio.paused) {
      try {
        await currentAudio.play();
        if (mountedRef.current) setState('playing');
      } catch (err: any) {
        if (mountedRef.current) {
          setState('error');
          setError(err.message || 'Failed to resume playback');
          if (errorTimerRef.current) clearTimeout(errorTimerRef.current);
          errorTimerRef.current = setTimeout(resetError, 3000);
        }
      }
      return;
    }

    stopCurrentAudio();
    currentKey = messageId;
    setState('loading');
    setError(null);

    let lastError: any = null;

    for (const baseUrl of BACKEND_URLS) {
      try {
        const response = await fetch(`${baseUrl}/tts`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ text }),
        });

        if (!response.ok) {
          if (response.status === 504) {
            throw new Error('TTS timed out. Please try again.');
          }
          if (response.status >= 500) {
            throw new Error('TTS server error. Please try again.');
          }
          const data = await response.json().catch(() => ({}));
          throw new Error(data.detail || `TTS request failed (${response.status})`);
        }

        const blob = await response.blob();
        const audioUrl = URL.createObjectURL(blob);
        const audio = new Audio(audioUrl);

        audio.onended = () => {
          URL.revokeObjectURL(audioUrl);
          if (currentKey === messageId) {
            currentKey = null;
            currentAudio = null;
          }
          if (mountedRef.current) {
            setState('idle');
          }
        };

        audio.onerror = () => {
          URL.revokeObjectURL(audioUrl);
          if (mountedRef.current) {
            if (currentKey === messageId) {
              currentKey = null;
              currentAudio = null;
              setState('error');
              setError('Audio playback failed');
              if (errorTimerRef.current) clearTimeout(errorTimerRef.current);
              errorTimerRef.current = setTimeout(resetError, 3000);
            }
          }
        };

        currentAudio = audio;
        await audio.play();
        if (mountedRef.current) {
          setState('playing');
        }
        return;

      } catch (err: any) {
        console.warn(`TTS failed for ${baseUrl}:`, err);
        lastError = err;
      }
    }

    if (mountedRef.current && lastError) {
      setState('error');
      setError(lastError.message || 'Failed to generate speech');
      if (errorTimerRef.current) clearTimeout(errorTimerRef.current);
      errorTimerRef.current = setTimeout(resetError, 3000);
    }
  }, [resetError]);

  const restart = useCallback(async () => {
    if (!currentAudio) {
      if (mountedRef.current) setState('idle');
      return;
    }
    currentAudio.currentTime = 0;
    try {
      await currentAudio.play();
      if (mountedRef.current) setState('playing');
    } catch (err: any) {
      if (mountedRef.current) {
        setState('error');
        setError(err.message || 'Failed to restart playback');
        if (errorTimerRef.current) clearTimeout(errorTimerRef.current);
        errorTimerRef.current = setTimeout(resetError, 3000);
      }
    }
  }, [resetError]);

  const stop = useCallback(() => {
    if (currentAudio) {
      currentAudio.pause();
    }
    if (mountedRef.current) {
      setState(currentAudio ? 'paused' : 'idle');
      setError(null);
    }
  }, []);

  return { state, error, play, stop, restart };
}
