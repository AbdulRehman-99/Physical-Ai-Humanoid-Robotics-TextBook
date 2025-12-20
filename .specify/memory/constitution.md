<!--
Sync Impact Report:
Version change: 1.1.0 -> 1.2.0
List of modified principles: Added Book RAG Chatbot component principles
Added sections: Book RAG Chatbot Component Principles
Removed sections: N/A
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- .specify/commands/sp.adr.toml: ⚠ pending
- .specify/commands/sp.analyze.toml: ⚠ pending
- .specify/commands/sp.checklist.toml: ⚠ pending
- .specify/commands/sp.clarify.toml: ⚠ pending
- .specify/commands/sp.constitution.toml: ⚠ pending
- .specify/commands/sp.git.commit_pr.toml: ⚠ pending
- .specify/commands/sp.implement.toml: ⚠ pending
- .specify/commands/sp.phr.toml: ⚠ pending
- .specify/commands/sp.plan.toml: ⚠ pending
- .specify/commands/sp.specify.toml: ⚠ pending
- .specify/commands/sp.tasks.toml: ⚠ pending
- README.md: ⚠ pending
- docs/quickstart.md: ⚠ pending
Follow-up TODOs: Ensure dependent templates are updated to align with the updated constitution principles.
-->
# High-Performance Technical Book Constitution

## Core Principles

### 1. Module Alignment
Align strictly with the "Physical AI & Humanoid Robotics" course modules and learning outcomes.

### 2. Academic Reliability
Maintain academically reliable, technical writing using verified robotics sources and primary documentation.

### 3. Terminology Consistency
Ensure consistent terminology across ROS 2, URDF/Xacro, controllers, Gazebo, Unity, Isaac Sim, Isaac ROS, Nav2, Whisper, and VLA systems.

### 4. Precise Citations
Cite official robotics manuals, ROS REP standards, peer-reviewed papers, and vendor docs with precise references.

### 5. Clarity and Reproducibility
Provide clean examples, reproducible steps, and accurate system descriptions for every concept.

### 6. Content Exclusions
Exclude speculation, filler, non-verifiable statements, or untested workflows.

### 7. Approved Toolchain
Use only the approved book-writing toolchain: Docusaurus v3.9.

### 8. Predictable Chapter Pattern
Enforce a predictable chapter pattern: Concept explanation → Diagram/code → Applied example → References.

### 9. Real-world Robotics Conventions
Ensure diagrams and code samples follow real-world robotics conventions and compile/run where applicable.

### 10. Accessibility and Structure
Require accessibility: consistent formatting, glossary of robotics terms, and modular chapter independence.

### 11. Docusaurus Best Practices
Follow official Docusaurus documentation https://docusaurus.io/docs for structure, navigation, deployment, theming, and content organization.

### 12. Chapter Independence
Guarantee that every chapter can be generated independently while still conforming to the global constitution.

### 13. Book Structure and Chapter Requirements
The book contain 8 chapters total across 4 modules. Each chapter will be at least 2500 words long having a headings, subheadings, paragraphs, and examples.

## Book RAG Chatbot Component Principles

### 14. Content-Based Answers Only
The Book RAG Chatbot MUST answer questions only from book content or user-selected text; NEVER use external knowledge. If the answer is not present in the provided content, respond with a clear "No answer found in the provided content."

### 15. Technology Stack Compliance
The Book RAG Chatbot MUST utilize the approved technology stack: FastAPI for the backend, OpenAI Agents/ChatKit for AI interactions, Qdrant for vector storage, and Neon for database management.

### 16. Accuracy and Transparency
The Book RAG Chatbot MUST clearly indicate when information is sourced from the book content versus when no relevant information is available, maintaining transparency about the limitations of its knowledge base.

## Governance
The Constitution establishes the foundational principles for the development and maintenance of the 'High-Performance Technical Book' and its associated Book RAG Chatbot. All content creation, bot responses, and modifications must adhere to these principles. Amendments to this constitution require a formal proposal, review by the project leads, and documented approval. Compliance with these principles will be reviewed regularly to ensure the academic and technical integrity of the book and chatbot. Any divergence from these guidelines must be explicitly justified and approved by the project leads.

**Version**: 1.2.0 | **Ratified**: 2025-12-09 | **Last Amended**: 2025-12-16
