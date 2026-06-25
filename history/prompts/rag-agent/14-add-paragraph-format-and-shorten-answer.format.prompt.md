---
id: 14
title: Add Paragraph Format and Shorten Answer
stage: refactor
date: 2026-06-25
surface: agent
model: deepseek-v4-flash-free
feature: rag-agent
branch: main
user: user
command: custom
labels: [formatting,paragraph,conciseness,plain-text]
links:
  spec: null
  plan: null
  tasks: null
  ticket: null
  adr: null
  pr: null
files:
 - Backend/Agent/agent.py
 - Backend/Connection/agent_wrapper.py
tests:
 - (none)
---

# Add Paragraph Format and Shorten Answer

## Summary
Updated formatting instructions in both services to include short paragraphs (~30 words) alongside headings and bullet points, and added conciseness constraint ("maximum 3 sections"). Replaced the old instruction that only specified headings + bullet points.

## Changes

### Instruction format (5 blocks across 2 files)
**Old:**
```
Structure your answer with clear headings, sub-headings, and bullet points in plain text. For example:
- Write a heading as: 'Introduction to ROS 2'
- ...
Do NOT use markdown symbols like #, ##, **, or backticks.
```

**New:**
```
Use plain text only. For each topic write:
- A heading on its own line
- 1 short paragraph (~30 words) explaining the concept
- 2-4 key bullet points starting with dash
Keep the whole answer concise with maximum 3 sections.
Do NOT use markdown symbols like #, ##, **, or backticks.
```

### Backend/Agent/agent.py
- Lines 58-63 (selected_text branch)
- Lines 75-80 (context branch)

### Backend/Connection/agent_wrapper.py
- Lines 126-131 (selected_text branch)
- Lines 149-154 (retrieved_chunks branch)
- Lines 163-168 (no-content branch)

### max_tokens
- Unchanged at 400

## Testing
- Python syntax verified for both files
- No logic changes — instruction text only
