import React, { useState, useEffect, useRef } from 'react';
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

    try {
      // Get selected text if any
      const selectedText = window.getSelection?.()?.toString() || '';

      // Call backend API
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: inputValue,
          selected_text: selectedText || null,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      // Add assistant message
      const assistantMessage: Message = {
        id: Date.now().toString(),
        content: data.response,
        role: 'assistant',
        timestamp: new Date(),
        sources: data.sources || [],
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error sending message:', error);

      const errorMessage: Message = {
        id: Date.now().toString(),
        content: 'Sorry, I encountered an error. Please try again.',
        role: 'assistant',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const toggleExpand = () => {
    setIsExpanded(!isExpanded);
  };

  return (
    <div className={`chatkit-container ${isExpanded ? 'expanded' : 'collapsed'}`}>
      <div className="chatkit-header" onClick={toggleExpand}>
        <h3>{isExpanded ? 'AI Assistant' : 'AI'}</h3>
        <span className="chatkit-toggle">{isExpanded ? '−' : '+'}</span>
      </div>

      {isExpanded && (
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
                            {source.text.substring(0, 100)}...
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
            <button type="submit" disabled={isLoading}>
              Send
            </button>
          </form>
        </div>
      )}
    </div>
  );
};

export default ChatKit;