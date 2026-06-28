import React, { useState, useEffect, useRef, useCallback, useMemo } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import ListenButton from '../Audio/ListenButton';
import { useAudio } from '../../hooks/useAudio';
import './ChatKit.css';

interface Message {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: Date;
  sources?: Array<{text: string, metadata?: any}>;
  isError?: boolean;
}

const isProduction = typeof window !== 'undefined' &&
  window.location.hostname !== 'localhost' &&
  !window.location.hostname.startsWith('127.');

const BACKEND_URLS = isProduction
  ? ['https://abdul-rehman-99-textbook.hf.space']
  : ['http://localhost:8000', 'http://localhost:8001', 'http://localhost:8002'];

const MAX_MESSAGES = 100;
const SESSION_EXPIRY_MS = 48 * 60 * 60 * 1000;

const WELCOME_MESSAGE: Message = {
  id: '1',
  content: "Hello! I'm your AI assistant for this book. Ask me anything about the content!",
  role: 'assistant',
  timestamp: new Date(),
};

function getSessionKey(sid: string): string {
  return `chatkit_session:${sid}`;
}

function saveSessionMessages(sid: string, msgs: Message[]): void {
  try {
    const trimmed = msgs.slice(-MAX_MESSAGES);
    localStorage.setItem(getSessionKey(sid), JSON.stringify(trimmed));
  } catch {}
}

function loadSessionMessages(sid: string): Message[] | null {
  try {
    const stored = localStorage.getItem(getSessionKey(sid));
    if (stored) {
      const parsed = JSON.parse(stored);
      if (Array.isArray(parsed) && parsed.length > 0) {
        return parsed.map((m: any) => ({ ...m, timestamp: new Date(m.timestamp) }));
      }
    }
  } catch {}
  return null;
}

function addSessionToHistory(sid: string, lastActive?: string): void {
  try {
    const order: Array<{id: string; lastActive: string}> = JSON.parse(
      localStorage.getItem('chatkit_sessions_order') || '[]'
    );
    const ts = lastActive || new Date().toISOString();
    const idx = order.findIndex(e => e.id === sid);
    if (idx >= 0) {
      order[idx].lastActive = ts;
    } else {
      order.unshift({ id: sid, lastActive: ts });
    }
    localStorage.setItem('chatkit_sessions_order', JSON.stringify(order));
  } catch {}
}

function updateSessionTimestamp(sid: string): void {
  try {
    const order: Array<{id: string; lastActive: string}> = JSON.parse(
      localStorage.getItem('chatkit_sessions_order') || '[]'
    );
    const ts = new Date().toISOString();
    const idx = order.findIndex(e => e.id === sid);
    if (idx >= 0) {
      order[idx].lastActive = ts;
      const [entry] = order.splice(idx, 1);
      order.unshift(entry);
    } else {
      order.unshift({ id: sid, lastActive: ts });
    }
    localStorage.setItem('chatkit_sessions_order', JSON.stringify(order));
  } catch {}
}

function sweepExpiredSessions(): void {
  try {
    const raw = localStorage.getItem('chatkit_sessions_order');
    if (!raw) return;
    const parsed = JSON.parse(raw);
    if (!Array.isArray(parsed)) return;
    const now = Date.now();
    const remaining: Array<{id: string; lastActive: string}> = [];
    for (const entry of parsed) {
      const id = typeof entry === 'string' ? entry : entry.id;
      const lastActive = typeof entry === 'string' ? now : new Date(entry.lastActive || now).getTime();
      if (now - lastActive >= SESSION_EXPIRY_MS) {
        localStorage.removeItem(getSessionKey(id));
      } else {
        remaining.push(typeof entry === 'string' ? { id: entry, lastActive: new Date(now).toISOString() } : entry);
      }
    }
    localStorage.setItem('chatkit_sessions_order', JSON.stringify(remaining));
  } catch {}
}

function getSessionList(): Array<{id: string; preview: string; lastActive: string}> {
  sweepExpiredSessions();
  try {
    const raw = localStorage.getItem('chatkit_sessions_order');
    if (!raw) return [];
    const parsed = JSON.parse(raw);
    if (!Array.isArray(parsed)) return [];

    return parsed.slice(0, 20).map((entry: any) => {
      const id = typeof entry === 'string' ? entry : entry.id;
      const lastActive = typeof entry === 'string' ? new Date().toISOString() : (entry.lastActive || new Date().toISOString());
      const msgs = loadSessionMessages(id);
      const userMsg = msgs?.find(m => m.role === 'user');
      const preview = userMsg ? userMsg.content.slice(0, 50) : '(empty)';
      return { id, preview, lastActive };
    });
  } catch { return []; }
}

function formatRelativeTime(iso: string): string {
  const diff = Date.now() - new Date(iso).getTime();
  const mins = Math.floor(diff / 60000);
  if (mins < 1) return 'Just now';
  if (mins < 60) return `${mins}m ago`;
  const hours = Math.floor(mins / 60);
  if (hours < 24) return `${hours}h ago`;
  const days = Math.floor(hours / 24);
  if (days < 7) return `${days}d ago`;
  return new Date(iso).toLocaleDateString('en-US', { month: 'short', day: 'numeric' });
}

const ChatKit: React.FC = () => {
  const [sessionId, setSessionId] = useState<string>(() => {
    try {
      const stored = localStorage.getItem('chatkit_active_session');
      if (stored) return stored;
    } catch {}
    const newId = crypto.randomUUID();
    try { localStorage.setItem('chatkit_active_session', newId); } catch {}
    return newId;
  });
  const [messages, setMessages] = useState<Message[]>(() => {
    try {
      const sid = localStorage.getItem('chatkit_active_session');
      if (sid) {
        const loaded = loadSessionMessages(sid);
        if (loaded) return loaded;
      }
    } catch {}
    return [WELCOME_MESSAGE];
  });
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isExpanded, setIsExpanded] = useState(false);
  const [showSessionList, setShowSessionList] = useState(false);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);
  const abortControllerRef = useRef<AbortController | null>(null);
  const { stop: stopAudio } = useAudio();

  useEffect(() => {
    const savedSid = localStorage.getItem('chatkit_active_session');
    const sid = savedSid || crypto.randomUUID();
    if (!savedSid) {
      localStorage.setItem('chatkit_active_session', sid);
    }
    if (sid !== sessionId) {
      setSessionId(sid);
      const loaded = loadSessionMessages(sid);
      if (loaded) setMessages(loaded);
    }

    const oldData = localStorage.getItem('chatkit_messages');
    if (oldData) {
      try {
        const parsed = JSON.parse(oldData);
        if (Array.isArray(parsed) && parsed.length > 0) {
          localStorage.setItem(getSessionKey(sid), oldData);
          addSessionToHistory(sid);
        }
      } catch {}
      localStorage.removeItem('chatkit_messages');
    }

    const raw = localStorage.getItem('chatkit_sessions_order');
    if (raw) {
      try {
        const parsed = JSON.parse(raw);
        if (Array.isArray(parsed) && parsed.length > 0 && typeof parsed[0] === 'string') {
          const migrated = parsed.map((id: string) => ({ id, lastActive: new Date().toISOString() }));
          localStorage.setItem('chatkit_sessions_order', JSON.stringify(migrated));
        }
      } catch {}
    }

    sweepExpiredSessions();
  }, []);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  useEffect(() => {
    const timer = setTimeout(() => {
      sweepExpiredSessions();
      saveSessionMessages(sessionId, messages);
      if (messages.length > 1) {
        updateSessionTimestamp(sessionId);
      }
    }, 500);
    return () => clearTimeout(timer);
  }, [messages, sessionId]);

  const tryStream = useCallback(async (
    message: string,
    selectedText: string,
  ): Promise<{ success: boolean; fullResponse?: string }> => {
    for (const baseUrl of BACKEND_URLS) {
      const url = `${baseUrl}/chat/stream`;
      try {
        const controller = new AbortController();
        abortControllerRef.current = controller;

        const response = await fetch(url, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            message,
            selected_text: selectedText || null,
            session_id: sessionId,
          }),
          signal: controller.signal,
        });

        if (!response.ok) continue;

        const reader = response.body?.getReader();
        if (!reader) continue;

        const decoder = new TextDecoder();
        let buffer = '';
        let accumulated = '';

        setMessages(prev => [...prev, {
          id: Date.now().toString(),
          content: '',
          role: 'assistant',
          timestamp: new Date(),
        }]);

        while (true) {
          const { done, value } = await reader.read();
          if (done) break;

          buffer += decoder.decode(value, { stream: true });
          const lines = buffer.split('\n');
          buffer = lines.pop() || '';

          for (const line of lines) {
            if (!line.trim()) continue;
            try {
              const data = JSON.parse(line);
              if (data.token) {
                accumulated += data.token;
                setMessages(prev => {
                  const updated = [...prev];
                  const last = updated[updated.length - 1];
                  if (last && last.role === 'assistant') {
                    last.content = accumulated;
                  }
                  return [...updated];
                });
              } else if (data.done) {
                if (data.session_id) {
                  setSessionId(data.session_id);
                  localStorage.setItem('chatkit_active_session', data.session_id);
                }
                return { success: true, fullResponse: accumulated };
              } else if (data.type === "off_topic") {
                setMessages(prev => {
                  const updated = [...prev];
                  const last = updated[updated.length - 1];
                  if (last && last.role === 'assistant') {
                    last.content = data.content || 'This question is not related to the book content.';
                    last.isError = true;
                  }
                  return [...updated];
                });
                return { success: true, fullResponse: data.content || '' };
              } else if (data.error) {
                return { success: false };
              }
            } catch {
              // ignore parse errors on partial lines
            }
          }
        }

        return { success: true, fullResponse: accumulated };
      } catch (err: any) {
        if (err.name === 'AbortError') {
          return { success: false };
        }
        console.warn(`Stream failed for ${url}:`, err);
      } finally {
        abortControllerRef.current = null;
      }
    }
    return { success: false };
  }, [sessionId]);

  const tryPost = useCallback(async (
    message: string,
    selectedText: string,
  ): Promise<{ success: boolean; response?: string; sources?: any[]; is_off_topic?: boolean }> => {
    for (const baseUrl of BACKEND_URLS) {
      const url = `${baseUrl}/chat`;
      try {
        const response = await fetch(url, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            message,
            selected_text: selectedText || null,
            session_id: sessionId,
          }),
        });

        if (response.ok) {
          const data = await response.json();
          return { success: true, response: data.response, sources: data.sources || [], is_off_topic: data.is_off_topic || false };
        }
      } catch (err) {
        console.warn(`POST failed for ${url}:`, err);
      }
    }
    return { success: false };
  }, [sessionId]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      content: inputValue,
      role: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    const currentInput = inputValue;
    setInputValue('');
    setIsLoading(true);

    const selectedText = window.getSelection()?.toString() || '';

    const streamResult = await tryStream(currentInput, selectedText);

    if (!streamResult.success) {
      const postResult = await tryPost(currentInput, selectedText);
      if (postResult.success) {
        if (postResult.is_off_topic) {
          setMessages(prev => [...prev, {
            id: Date.now().toString(),
            content: postResult.response || '',
            role: 'assistant',
            timestamp: new Date(),
            isError: true,
          }]);
        } else {
          setMessages(prev => [...prev, {
            id: Date.now().toString(),
            content: postResult.response || '',
            role: 'assistant',
            timestamp: new Date(),
            sources: postResult.sources,
          }]);
        }
      } else {
        setMessages(prev => [...prev, {
          id: Date.now().toString(),
          content: 'Sorry, I couldn\'t connect to any backend service. Please ensure your backend is running.',
          role: 'assistant',
          timestamp: new Date(),
        }]);
      }
    }

    setIsLoading(false);
  };

  const handleRefresh = () => {
    stopAudio();
    sweepExpiredSessions();
    if (messages.length > 1) {
      saveSessionMessages(sessionId, messages);
      updateSessionTimestamp(sessionId);
    }
    const newId = crypto.randomUUID();
    setSessionId(newId);
    localStorage.setItem('chatkit_active_session', newId);
    setMessages([{ ...WELCOME_MESSAGE, timestamp: new Date() }]);
    setShowSessionList(false);
  };

  const handleLoadSession = (sid: string) => {
    stopAudio();
    sweepExpiredSessions();
    const msgs = loadSessionMessages(sid);
    if (!msgs || msgs.length === 0) return;
    if (sessionId !== sid && messages.length > 1) {
      saveSessionMessages(sessionId, messages);
      updateSessionTimestamp(sessionId);
    }
    setSessionId(sid);
    localStorage.setItem('chatkit_active_session', sid);
    setMessages(msgs);
    setShowSessionList(false);
  };

  const handleDeleteSession = (e: React.MouseEvent, sid: string) => {
    e.stopPropagation();
    try {
      localStorage.removeItem(getSessionKey(sid));
      const order: Array<{id: string; lastActive: string}> = JSON.parse(
        localStorage.getItem('chatkit_sessions_order') || '[]'
      );
      const filtered = order.filter(e => e.id !== sid);
      localStorage.setItem('chatkit_sessions_order', JSON.stringify(filtered));
    } catch {}
  };

  const toggleExpand = () => {
    if (isExpanded) stopAudio();
    setIsExpanded(!isExpanded);
  };

  const sessionList = getSessionList();

  const lastAssistantMessage = useMemo(() => {
    const assistantMsgs = messages.filter(
      m => m.role === 'assistant' && !m.isError && m.content
    );
    return assistantMsgs.length > 0 ? assistantMsgs[assistantMsgs.length - 1] : null;
  }, [messages]);

  return (
    <div className={`chatkit-wrapper ${isExpanded ? 'expanded' : 'collapsed'}`}>
      {!isExpanded && (
        <button className="chatkit-launcher" onClick={toggleExpand} aria-label="Open AI Assistant">
          <div className="launcher-icon">
            <svg xmlns="http://www.w3.org/2000/svg" width="28" height="28" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
              <path d="M12 8V4H8"/>
              <rect width="16" height="12" x="4" y="8" rx="2"/>
              <path d="M2 14h2"/>
              <path d="M20 14h2"/>
              <path d="M15 13v2"/>
              <path d="M9 13v2"/>
            </svg>
          </div>
          <div className="launcher-glow"></div>
        </button>
      )}

      {isExpanded && (
        <div className="chatkit-window">
          <div className="chatkit-header">
            <div className="header-info">
              <div className="status-indicator"></div>
              <h3>AI Assistant</h3>
            </div>
            <div className="chatkit-header-actions">
              {lastAssistantMessage && (
                <ListenButton
                  text={lastAssistantMessage.content}
                  messageId={lastAssistantMessage.id}
                  variant="header"
                />
              )}
              <button className="chatkit-icon-btn" onClick={() => setShowSessionList(true)} aria-label="Chat History" title="Chat History">
                <svg xmlns="http://www.w3.org/2000/svg" width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                  <circle cx="12" cy="12" r="10"/>
                  <polyline points="12 6 12 12 16 14"/>
                </svg>
              </button>
              <button className="chatkit-icon-btn" onClick={handleRefresh} aria-label="New Conversation" title="New Conversation">
                <svg xmlns="http://www.w3.org/2000/svg" width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                  <polyline points="23 4 23 10 17 10"/>
                  <polyline points="1 20 1 14 7 14"/>
                  <path d="M3.51 9a9 9 0 0 1 14.85-3.36L23 10M1 14l4.64 4.36A9 9 0 0 0 20.49 15"/>
                </svg>
              </button>
              <button className="chatkit-close" onClick={toggleExpand} aria-label="Close Assistant">
                <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                  <line x1="18" y1="6" x2="6" y2="18"></line>
                  <line x1="6" y1="6" x2="18" y2="18"></line>
                </svg>
              </button>
            </div>
          </div>

          <div className="chatkit-content">
            {showSessionList ? (
              <div className="chatkit-session-list">
                <div className="session-list-header">
                  <h4>Conversations</h4>
                  <button onClick={() => setShowSessionList(false)}>Back</button>
                </div>
                {sessionList.length > 0 ? (
                  sessionList.map(s => (
                    <div key={s.id} className={`session-item${s.id === sessionId ? ' active' : ''}`} onClick={() => handleLoadSession(s.id)}>
                      <div className="session-info">
                        <div className="session-preview">{s.preview}</div>
                        <div className="session-time">{formatRelativeTime(s.lastActive)}</div>
                      </div>
                      <button className="session-delete" onClick={(e) => handleDeleteSession(e, s.id)} aria-label="Delete conversation">
                        <svg xmlns="http://www.w3.org/2000/svg" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                          <polyline points="3 6 5 6 21 6"/>
                          <path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"/>
                        </svg>
                      </button>
                    </div>
                  ))
                ) : (
                  <div className="session-empty">No saved conversations</div>
                )}
              </div>
            ) : (
              <>
                <div className="chatkit-messages">
                  {messages.map((message) => (
                    <div
                      key={message.id}
                      className={`chatkit-message ${message.role}${message.isError ? ' error' : ''}`}
                    >
                      <div className="chatkit-message-content">
                        {message.content}
                        {message.role === 'assistant' && !message.isError && message.content && (
                          <div className="message-listen">
                            <ListenButton text={message.content} messageId={message.id} variant="message" />
                          </div>
                        )}
                        {message.sources && message.sources.length > 0 && (
                          <div className="chatkit-sources">
                            <details>
                              <summary>Sources</summary>
                              {message.sources.map((source, index) => (
                                <div key={index} className="chatkit-source">
                                    {source.text ? source.text.substring(0, 100) : 'No content available'}...
                                </div>
                              ))}
                            </details>
                          </div>
                        )}
                      </div>
                    </div>
                  ))}
                  {isLoading && messages[messages.length - 1]?.role === 'user' && (
                    <div className="chatkit-message assistant">
                      <div className="chatkit-message-content">
                        <div className="typing-indicator">
                          <span></span>
                          <span></span>
                          <span></span>
                        </div>
                      </div>
                    </div>
                  )}
                  <div ref={messagesEndRef} />
                </div>

                <form className="chatkit-input-form" onSubmit={handleSubmit}>
                  <input
                    type="text"
                    value={inputValue}
                    onChange={(e) => setInputValue(e.target.value)}
                    placeholder="Ask about the book content..."
                    disabled={isLoading}
                  />
                  <button type="submit" disabled={isLoading} className="send-button">
                    <svg xmlns="http://www.w3.org/2000/svg" width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                      <line x1="22" y1="2" x2="11" y2="13"></line>
                      <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
                    </svg>
                  </button>
                </form>
              </>
            )}
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatKit;
