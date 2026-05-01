# Module Cards - Documentation Links

## Overview
This document lists the module cards implemented on the Docusaurus homepage and their corresponding documentation routes.

## Module Cards Structure

### Module 1: Fundamentals
- **Title**: Module 1: Fundamentals
- **Description**: Introduction to ROS (Robot Operating System), basic concepts, and foundational principles of robotics development.
- **Route**: `/docs/module-1-ros/ch1-foundations-humanoid-basics`
- **Color**: Blue (`#3b82f6`)

### Module 2: Perception and Navigation
- **Title**: Module 2: Perception and Navigation
- **Description**: Advanced simulation techniques, sensor integration, and navigation algorithms for autonomous robot movement.
- **Route**: `/docs/module-2-simulation/ch3-gazebo-digital-twin-physics`
- **Color**: Green (`#10b981`)

### Module 3: Vision-Language-Action Systems
- **Title**: Module 3: Vision-Language-Action Systems
- **Description**: NVIDIA Isaac ecosystem, computer vision, and AI-driven robotic control systems for intelligent automation.
- **Route**: `/docs/module-3-nvidia-isaac/ch5-isaac-sim-perception-synthetic-data`
- **Color**: Purple (`#8b5cf6`)

### Module 4: Advanced Topics
- **Title**: Module 4: Advanced Topics
- **Description**: Cutting-edge integration techniques, advanced AI models, and complex robotic system architectures.
- **Route**: `/docs/module-4-advanced-integration/ch7-whisper-llm-vla-planning`
- **Color**: Amber (`#f59e0b`)

## Implementation Details

### Components Created
1. `src/components/ModuleCards/index.tsx` - Main ModuleCards component
2. `src/components/ModuleCards/styles.module.css` - Styling for module cards

### Changes Made
1. Updated `src/components/HomepageFeatures/index.tsx` to include the ModuleCards component
2. Added responsive design for mobile, tablet, and desktop
3. Implemented accessibility features
4. Used Docusaurus Link component for proper routing

### Features
- Responsive grid layout (4 columns on desktop, 2 on tablet, 1 on mobile)
- Hover effects with elevation and color transitions
- Accessible navigation with keyboard support
- Proper semantic HTML structure
- Reduced motion support for accessibility
- Gradient backgrounds and modern card design

## Navigation Routes
All links use the standard Docusaurus routing system:
- `@docusaurus/Link` for proper SPA navigation
- Predefined documentation routes from the sidebar configuration
- Proper anchor behavior and focus management

## Accessibility Compliance
- WCAG 2.1 AA compliant design
- Proper focus management
- Reduced motion support
- Semantic HTML structure
- ARIA-compliant navigation