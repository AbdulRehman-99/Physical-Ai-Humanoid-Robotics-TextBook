# Physical AI & Humanoid Robotics Book

Welcome to the "Physical AI & Humanoid Robotics" book project! This is a comprehensive technical book on the intersection of artificial intelligence and humanoid robotics.

## Overview

This book explores the cutting-edge field of Physical AI and its application to humanoid robotics. It covers topics including:

- Physical AI principles and implementation
- Humanoid robotics design and control
- AI integration for autonomous behavior
- Simulation environments for robotics
- NVIDIA Isaac ROS integration
- Advanced integration techniques

## RAG (Retrieval-Augmented Generation) Integration

This book features an advanced RAG system that allows you to interact with the book content through an AI-powered chat interface.

### Features

- **AI-Powered Q&A**: Ask questions about the book content and get AI-generated answers
- **Context-Aware Responses**: The system understands the context of your questions
- **Source Citations**: Responses include references to specific sections of the book
- **Selected Text Context**: When you select text in the book, the AI can focus on that specific content
- **Academic Reliability**: Responses are grounded in the book content with proper citations

### Architecture

The RAG system consists of:

1. **Frontend**: Docusaurus-based book interface with integrated ChatKit UI
2. **Backend**: FastAPI server handling chat requests and AI processing
3. **Vector Database**: Qdrant for efficient content retrieval
4. **AI Model**: Google Gemini for natural language understanding and generation

### How It Works

1. Book content is embedded and stored in a Qdrant vector database
2. When you ask a question, the system retrieves relevant book sections
3. The AI model generates responses based on the retrieved context
4. Answers are presented with source citations in the chat interface

### Technical Stack

- **Frontend**: Docusaurus, React
- **Backend**: FastAPI, Python
- **Vector Database**: Qdrant Cloud
- **AI Model**: Google Gemini
- **Embeddings**: Multilingual embedding models

### Usage

1. Navigate to the book at `http://localhost:3000/`
2. Look for the ChatKit widget on the right side of the page
3. Click to expand and start asking questions about the book content
4. Select text in the book to provide specific context to the AI

### API Endpoints

- **Chat Endpoint**: `POST /chat` - Process user queries and return AI responses
- **Health Check**: `GET /health` - Verify service availability

### Security & Rate Limiting

- Rate limiting: 100 requests per minute per IP address
- Input sanitization for security
- CORS configured for web browser access

## Getting Started

1. Start the backend service:
   ```bash
   cd Backend
   python main.py
   ```

2. Start the frontend:
   ```bash
   cd Frontend
   npx docusaurus start
   ```

3. Visit `http://localhost:3000/` to access the book with the integrated chat interface.

## License

This project is licensed under the terms specified in the project documentation.