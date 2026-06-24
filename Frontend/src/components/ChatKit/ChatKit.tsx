import React, { useState, useEffect, useRef, useCallback } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import './ChatKit.css';

interface Message {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: Date;
  sources?: Array<{text: string, metadata?: any}>;
}

const BACKEND_URLS = [
  'http://localhost:8000',
  'http://localhost:8002',
  'https://abdul-rehman-99-textbook.hf.space',
];

const ChatKit: React.FC = () => {
  const [messages, setMessages] = useState<Message[]>([
    {
      id: '1',
      content: 'Hello! I\'m your AI assistant for this book. Ask me anything about the content!',
      role: 'assistant',
      timestamp: new Date(),
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isExpanded, setIsExpanded] = useState(false);
  const [showHistory, setShowHistory] = useState(false);
  const [sessionId, setSessionId] = useState<string | null>(() => `session_${Date.now()}`);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);
  const abortControllerRef = useRef<AbortController | null>(null);

  const displayMessages = showHistory
    ? messages
    : messages.slice(-6);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages, displayMessages.length]);

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
                }
                return { success: true, fullResponse: accumulated };
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
  ): Promise<{ success: boolean; response?: string; sources?: any[] }> => {
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
          return { success: true, response: data.response, sources: data.sources || [] };
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

    // Try streaming first, fall back to POST
    const streamResult = await tryStream(currentInput, selectedText);

    if (!streamResult.success) {
      const postResult = await tryPost(currentInput, selectedText);
      if (postResult.success) {
        setMessages(prev => [...prev, {
          id: Date.now().toString(),
          content: postResult.response || '',
          role: 'assistant',
          timestamp: new Date(),
          sources: postResult.sources,
        }]);
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

  const toggleExpand = () => {
    setIsExpanded(!isExpanded);
  };

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
            <button className="chatkit-close" onClick={toggleExpand} aria-label="Close Assistant">
              <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                <line x1="18" y1="6" x2="6" y2="18"></line>
                <line x1="6" y1="6" x2="18" y2="18"></line>
              </svg>
            </button>
          </div>

          <div className="chatkit-content">
            <div className="chatkit-history-bar">
              <button
                className="chatkit-history-toggle"
                onClick={() => setShowHistory(!showHistory)}
              >
                {showHistory ? 'Show last 6' : 'Show all'}
              </button>
            </div>
            <div className="chatkit-messages">
              {displayMessages.map((message) => (
                <div
                  key={message.id}
                  className={`chatkit-message ${message.role}`}
                >
                  <div className="chatkit-message-content">
                    {message.content}
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
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatKit;
