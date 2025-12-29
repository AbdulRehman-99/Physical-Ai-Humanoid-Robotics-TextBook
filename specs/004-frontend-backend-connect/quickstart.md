# Quickstart Guide: Frontend ↔ Backend Connection

**Feature**: Connect Frontend ↔ Backend with ChatKit
**Created**: 2025-12-25
**Status**: Draft
**Author**: Claude

## Overview

This guide provides instructions for setting up, running, and testing the ChatKit ↔ FastAPI connection that enables users to ask questions about book content and receive AI-generated responses.

## Prerequisites

### System Requirements
- Python 3.9 or higher
- Node.js 16 or higher (for frontend)
- Docker (for Qdrant if running locally)
- Access to OpenAI Agent SDK with Gemini adapter

### Environment Setup
1. Backend dependencies: `pip install fastapi uvicorn openai python-dotenv qdrant-client`
2. Frontend dependencies: `npm install @chatkit/chatkit`
3. Environment variables:
   - `QDRANT_URL`: URL to Qdrant instance
   - `QDRANT_API_KEY`: API key for Qdrant (if required)
   - `OPENAI_API_KEY`: API key for OpenAI Agent SDK
   - `GEMINI_ADAPTER_CONFIG`: Configuration for Gemini adapter

## Local Development Setup

### 1. Clone and Navigate
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Backend Setup
```bash
cd Backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 3. Environment Variables
Create `.env` file in Backend directory:
```env
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your_qdrant_api_key
OPENAI_API_KEY=your_openai_api_key
GEMINI_ADAPTER_CONFIG=your_gemini_config
```

### 4. Start Backend Services
```bash
# Start Qdrant (if running locally)
docker run -p 6333:6333 -v ./qdrant_storage:/qdrant/storage:z qdrant/qdrant

# Start FastAPI server
cd Backend
uvicorn Connection.api:app --reload --port 8000
```

### 5. Frontend Setup
```bash
cd Frontend
npm install
npm start
```

## API Usage

### Testing the Chat Endpoint

#### Using curl:
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "Explain inverse kinematics",
    "selected_text": null
  }'
```

#### Using selected text:
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What does this mean?",
    "selected_text": "Inverse kinematics is the mathematical process of calculating the joint angles required to position the end effector of a robotic arm at a desired location."
  }'
```

### Expected Response Format
```json
{
  "response": "Response text from the AI agent",
  "sources": ["Chapter references or book content that informed the response"]
}
```

## Running Tests

### Backend Tests
```bash
cd Backend
python -m pytest Connection/tests/
```

### Integration Tests
```bash
cd Backend
python -m pytest Connection/tests/integration/
```

## Key Components

### Connection Module Structure
```
Backend/
├── Connection/
│   ├── __init__.py
│   ├── api.py                 # FastAPI endpoints
│   ├── context_switcher.py    # Logic for selected_text vs Qdrant
│   ├── agent_wrapper.py       # Agent invocation and configuration
│   ├── response_formatter.py  # ChatKit-compatible response formatting
│   └── models.py              # Data models for requests/responses
```

### Key Files to Understand
- `Connection/api.py`: Main API endpoint implementation
- `Connection/context_switcher.py`: Logic that decides between selected_text and Qdrant retrieval
- `Connection/agent_wrapper.py`: Interface with OpenAI Agent SDK and Gemini adapter
- `Connection/response_formatter.py`: Formats responses for ChatKit compatibility

## Development Workflow

### 1. Making Changes to API Logic
1. Modify the relevant file in `Backend/Connection/`
2. Update tests in `Backend/Connection/tests/`
3. Run tests: `python -m pytest Backend/Connection/tests/`
4. Test manually using curl or frontend

### 2. Testing Context Switching
```python
# Test selected_text override
response = context_switcher.determine_context(
    user_message="What does this mean?",
    selected_text="This is the selected text that should be used."
)
# Should return selected_text as context, ignoring Qdrant

# Test Qdrant fallback
response = context_switcher.determine_context(
    user_message="Explain inverse kinematics",
    selected_text=None
)
# Should perform Qdrant retrieval
```

### 3. Testing Agent Integration
1. Ensure Qdrant is running and populated with book content
2. Verify OpenAI Agent SDK is properly configured with Gemini adapter
3. Test end-to-end flow from API request to agent response

## Common Issues and Troubleshooting

### Qdrant Connection Issues
- Ensure Qdrant is running and accessible at the configured URL
- Check API key if authentication is required
- Verify the collection exists and contains book content

### Agent Configuration Issues
- Verify OpenAI API key is valid and has proper permissions
- Check that Gemini adapter is properly configured
- Ensure sufficient quota for API calls

### Response Time Issues
- Monitor Qdrant query performance
- Check agent response times
- Consider caching frequently asked questions

## Deployment

### Production Environment Variables
```env
QDRANT_URL=production_qdrant_url
QDRANT_API_KEY=production_qdrant_api_key
OPENAI_API_KEY=production_openai_api_key
GEMINI_ADAPTER_CONFIG=production_gemini_config
ENVIRONMENT=production
LOG_LEVEL=info
```

### Deployment Commands
```bash
# Build and deploy backend
cd Backend
gunicorn Connection.api:app -w 4 -k uvicorn.workers.UvicornWorker

# Frontend deployment (Docusaurus)
cd Frontend
npm run build
# Serve built files with your preferred web server
```

## Next Steps

1. Review the [implementation plan](plan.md) for detailed architecture
2. Check the [data models](data-model.md) for detailed field definitions
3. Explore the [API contracts](contracts/chat-api.yaml) for complete specification
4. Run the end-to-end tests to verify the complete flow