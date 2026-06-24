---
id: 7
title: Analyze Project and Switch Branch
stage: misc
date: 2026-06-25
surface: agent
model: deepseek-v4-flash-free
feature: rag-agent
branch: update-agent-framework
user: user
command: custom
labels: [analysis,project-exploration,branching,git]
links:
  spec: null
  plan: null
  tasks: null
  ticket: null
  adr: null
  pr: null
files:
 - Backend/Agent/agent.py
 - Backend/Agent/main.py
 - Backend/Agent/adapter.py
 - Backend/Agent/settings.py
 - Backend/Connection/agent_wrapper.py
 - Backend/pyproject.toml
tests: []
---

## Prompt

analyze all files and folder and store the context of the whole project in your memory

Tell me which agentic framework is being used for the agent implementation in this project.

continue the worl

first create a new branch, and switch to that branch

yes go ahead

update-agent-framework

## Response snapshot

### Project Analysis
Comprehensive full-project analysis completed via exploration agent. The project is a **Physical AI & Humanoid Robotics Textbook** with dual purpose:
1. An 8-chapter Docusaurus v3.9 textbook across 4 modules
2. A RAG-powered AI assistant using Qdrant vector DB + Cohere embeddings + Gemini 2.5 Flash

**Directory structure** 5 main feature specs under SDD methodology, Backend/ with 4 sub-services (Ingestion, Retrieval, Agent, Connection), Frontend/ with Docusaurus content and ChatKit component.

### Agent Framework Answer
Two agent implementations found:
1. **Backend/Agent/** (port 8001) — Uses **LiteLLM** (`from litellm import completion`). The `GeminiAdapter` wraps LiteLLM to provide an OpenAI-compatible interface. The `RAGAgent` class docstring references "OpenAI Agent SDK v0.6 pattern" architecturally.
2. **Backend/Connection/** (port 8000) — Uses **google-generativeai** directly.
Note: `openai-agents` is listed in `pyproject.toml` as a dependency but never imported.

### Branch Created
Created and switched to `update-agent-framework` branch from `main`.

## Outcome

Full project context stored in memory. Branch `update-agent-framework` created and checked out.
