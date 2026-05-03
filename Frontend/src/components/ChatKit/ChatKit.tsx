import React, { useState, useEffect, useRef } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import './ChatKit.css';

interface Message {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: Date;
  sources?: Array<{text: string, metadata?: any}>;
}

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
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      content: inputValue,
      role: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    // List of backend URLs to try (local development and production)
    const backendUrls = [
      'http://localhost:8000/chat',
      'http://localhost:8002/chat',
      'https://abdul-rehman-99-textbook.hf.space/chat'
    ];

    let success = false;
    let responseData = null;

    for (const url of backendUrls) {
      try {
        console.log(`Attempting to connect to backend at: ${url}`);
        const selectedText = window.getSelection?.()?.toString() || '';
        
        const response = await fetch(url, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            message: inputValue,
            selected_text: selectedText || null,
          }),
        });

        if (response.ok) {
          responseData = await response.json();
          success = true;
          break; // Exit loop on success
        }
      } catch (error) {
        console.warn(`Failed to connect to ${url}:`, error);
      }
    }

    if (success && responseData) {
      // Add assistant message
      const assistantMessage: Message = {
        id: Date.now().toString(),
        content: responseData.response,
        role: 'assistant',
        timestamp: new Date(),
        sources: responseData.sources || [],
      };
      setMessages(prev => [...prev, assistantMessage]);
    } else {
      const errorMessage: Message = {
        id: Date.now().toString(),
        content: 'Sorry, I couldn\'t connect to any backend service. Please ensure your backend is running on port 8000 or 8002.',
        role: 'assistant',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
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
            <div className="chatkit-messages">
              {messages.map((message) => (
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
              {isLoading && (
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