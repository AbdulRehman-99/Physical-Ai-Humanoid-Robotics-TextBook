---
id: 15
title: Add Concrete Format Example with Line Breaks
stage: refactor
date: 2026-06-25
surface: agent
model: deepseek-v4-flash-free
feature: rag-agent
branch: main
user: user
command: custom
labels: [formatting,example,line-breaks,paragraph]
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

# Add Concrete Format Example with Line Breaks

## Summary
Replaced abstract formatting rules with a concrete visual example showing exact line-break structure. The model was ignoring the abstract instruction ("A heading on its own line") and concatenating heading + paragraph on one line. The concrete example demonstrates heading → blank line → paragraph → blank line → bullet points each on their own line.

## Changes

### Format instruction (5 blocks across 2 files)

**Old (abstract rules):**
```
Use plain text only. For each topic write:
- A heading on its own line
- 1 short paragraph (~30 words) explaining the concept
- 2-4 key bullet points starting with dash
```

**New (concrete example):**
```
Use plain text only. Format each section like this example:

Introduction to ROS 2

ROS 2 is the next-generation robot operating system designed for real-time and distributed systems. It improves upon previous versions with better security.

- Nodes communicate via topics and services
- Supports multiple robots in one system
- Uses DDS middleware for reliability
```

### Files modified
- `Backend/Agent/agent.py` — 2 blocks (selected_text branch line 58, context branch line 81)
- `Backend/Connection/agent_wrapper.py` — 3 blocks (selected_text line 126, retrieved_chunks line 155, no-content line 175)

### Constraints kept
- "Keep the whole answer concise with maximum 3 sections"
- "Do NOT use markdown symbols like #, ##, **, or backticks"
- max_tokens=400 unchanged

## Testing
- Python syntax verified for both files
