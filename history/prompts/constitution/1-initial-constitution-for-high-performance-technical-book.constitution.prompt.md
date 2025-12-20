---
id: 1
title: Initial Constitution for High-Performance Technical Book (Updated)
stage: constitution
date: 2025-12-15
surface: command_line
model: Claude Sonnet 4.5
feature: humanoid-robotics-book
branch: main
user: human
command: "/sp.constitution Create a Project Constitution for a high-performance technical book: Align strictly with the “Physical AI & Humanoid Robotics” course modules and learning outcomes. Maintain academically reliable, technical writing using verified robotics sources and primary documentation. Ensure consistent terminology across ROS 2, URDF/Xacro, controllers, Gazebo, Unity, Isaac Sim, Isaac ROS, Nav2, Whisper, and VLA systems. Cite official robotics manuals, ROS REP standards, peer-reviewed papers, and vendor docs with precise references. Provide clean examples, reproducible steps, and accurate system descriptions for every concept. Exclude speculation, filler, non-verifiable statements, or untested workflows. Use only the approved book-writing toolchain: Docusaurus v3.9. Enforce a predictable chapter pattern: Concept explanation → Diagram/code → Applied example → References. Ensure diagrams and code samples follow real-world robotics conventions and compile/run where applicable. Require accessibility: consistent formatting, glossary of robotics terms, and modular chapter independence. Follow official Docusaurus documentation for structure, navigation, deployment, theming, and content organization. Guarantee that every chapter can be generated independently while still conforming to the global constitution. Each chapter will be at least 2500 words long having a heading, subheading, paragraph, and example."
labels: ["constitution", "project-setup", "update"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - description: Constitution version updated from 1.0.0 to 1.1.0
    status: passed
  - description: Chapter length requirement updated to 2500 words
    status: passed
---

## Prompt

/sp.constitution Establish the Project Constitution for a top-tier technical guidebook:
Adhere rigidly to the syllabus and objectives of the “Physical AI & Humanoid Robotics” course.
Uphold scholarly, technical rigor by utilizing authenticated robotics resources and primary documentation.
Standardize vocabulary across ROS 2, URDF/Xacro, controllers, Gazebo, Unity, Isaac Sim, Isaac ROS, Nav2, Whisper, and VLA systems.
Reference authoritative sources including official manuals, ROS REP standards, academic papers, and vendor documentation.
Deliver clear illustrations, repeatable procedures, and precise system details for every topic.
Omit conjecture, redundancy, unverified claims, or unproven workflows.
Utilize the exclusive toolchain: Docusaurus v3.9.
Mandate a consistent chapter structure: Concept definition → Visual/Script → Practical application → Citations.
Validate that schematics and scripts adhere to industry standards and are functional where applicable.
Prioritize readability through uniform formatting, a robotics glossary, and self-contained chapter modules.
Comply with official Docusaurus guidelines for architecture, navigation, deployment, styling, and organization.
Ensure modular generation capability for each chapter within the global framework.
Enforce a minimum length of 2500 words per chapter, comprising headings, subheadings, text blocks, and examples.

## Response snapshot

The constitution for the 'High-Performance Technical Book' has been created and updated at `.specify/memory/constitution.md`. New Version: 1.1.0. This is the updated version of the project constitution, incorporating all the principles and guidelines provided for the 'High-Performance Technical Book', with the chapter length requirement updated from 1000 to 2500 words to align with the 70/30 theory/examples ratio requirement.

## Outcome

- ✅ Impact: Updated the project constitution to version 1.1.0 with increased chapter length requirement.
- 🧪 Tests: Constitution version and chapter length requirement validated.
- 📁 Files: Constitution file updated: .specify/memory/constitution.md
- 🔁 Next prompts: Update dependent templates to align with new constitution.
- 🧠 Reflection: Successfully updated constitution to align with project requirements for more comprehensive theoretical content (70% theory, 30% code examples) with chapters of 2500-4000 words.

## Evaluation notes (flywheel)

- Failure modes observed: None - update completed successfully
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Ensure dependent templates are updated to align with the updated constitution principles.
