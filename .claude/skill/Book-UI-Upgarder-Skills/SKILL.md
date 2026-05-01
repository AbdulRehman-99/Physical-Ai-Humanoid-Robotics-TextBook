---
name: modern-docusaurus-ui
description: Upgrade the Docusaurus site UI with modern design patterns and better responsiveness without altering logic or chat UI.
---

# Docusaurus Modern UI Upgrade

## Objective
Enhance UI/UX of a Docusaurus documentation site with:

1. **Improved layout & visual hierarchy**
2. **Better typography & spacing**
3. **Modern color palette and themes**
4. **Responsive layout across devices**
5. **Navbar, footer, hero section, docs pages refinement**
6. **Maintain existing logic and existing ChatKit UI intact**

---

## Design Guidelines

### 🎨 Colors & Theme
- Apply a modern, pleasing color palette
  - Primary: Soft gradient or brand color
  - Secondary: Calm neutral tones
  - Text: High contrast for readability

### ✨ Typography
- Use modern font stack (e.g. Inter, Poppins)
- Improve heading hierarchy
- Increase line spacing for readability

---

## UI Components & Sections

### 📌 Navbar
- Clean, sticky navbar
- Add hover animations to links
- Include a contrasting background for clarity

### 📌 Hero/Landing Section
- Stylized hero with modern buttons
- Should highlight key action (Start Reading)

### 📌 Modules Section (Cards)
- Convert module links into card components
- Add hover shadows and smooth transitions

### 📌 Footer
- Simplify footer layout
- Add social icons with consistent design

---

## Technical Implementation

### 📍 Base
- Use `docusaurus.config.js` for theme config: colorMode, navbar, footer, and presets. :contentReference[oaicite:1]{index=1}

### 📍 Custom CSS
Place custom styles in `/src/css/custom.css`  
- Override Infima variables (default styling system). :contentReference[oaicite:2]{index=2}

### 📍 Swizzle Components
For deeper structural changes (without logic changes):
```sh
npx docusaurus swizzle @docusaurus/theme-classic Navbar
npx docusaurus swizzle @docusaurus/theme-classic Footer
