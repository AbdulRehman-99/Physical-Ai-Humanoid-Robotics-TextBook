# Favicon Setup Guide

## Current Setup
The website now uses a custom SVG favicon representing the Physical AI & Humanoid Robotics theme.

## Recommended Favicon Files
For optimal browser compatibility, you should generate these files:

### Standard Sizes
- `favicon-16x16.png` - 16x16 pixels (standard size)
- `favicon-32x32.png` - 32x32 pixels (larger displays)
- `favicon.ico` - Multi-size ICO file (16x16, 32x32, 48x48)

### Apple Touch Icons
- `apple-touch-icon.png` - 180x180 pixels

### Android Manifest
- `android-chrome-192x192.png` - 192x192 pixels
- `android-chrome-512x512.png` - 512x512 pixels

## How to Generate
1. Use an online favicon generator like favicon.io or realfavicongenerator.net
2. Upload your robot-themed image (use the SVG provided)
3. Download the generated files
4. Place them in the `static/img/` directory
5. Update the docusaurus.config.ts file with proper paths

## Alternative Configuration
If you generate the standard favicon files, you can update the config to:

```typescript
  themeConfig: {
    // ... other config
    metadata: [
      {name: 'keywords', content: 'physical ai, humanoid robotics, robotics, ai, textbook'},
      {name: 'theme-color', content: '#2563eb'},
    ],
  },
```

## Current SVG Favicon
The current configuration uses `img/favicon-robot.svg` which is a robot-themed icon representing the Physical AI & Humanoid Robotics book theme.