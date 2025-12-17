# Quickstart Guide: Physical AI & Humanoid Robotics Book

## Overview
This guide provides a quick setup and development workflow for contributing to the Physical AI & Humanoid Robotics book project.

## Prerequisites
- Node.js (version 18 or higher)
- npm or yarn package manager
- Git
- A GitHub account

## Setup Instructions

### 1. Clone the Repository
```bash
git clone [repository-url]
cd my-project
```

### 2. Install Dependencies
```bash
npm install
```

### 3. Start Local Development Server
```bash
npm start
```
This will start a local development server at `http://localhost:3000` with live reloading.

## Content Creation Workflow

### 1. Create a New Chapter
1. Navigate to the appropriate module directory:
   - For Module 1 (ROS 2): `docs/docs/module-1/`
   - For Module 2 (Digital Twin): `docs/docs/module-2/`
   - For additional content: `docs/docs/additional/`

2. Create a new Markdown file with a descriptive name:
```bash
# Example for a new ROS 2 chapter
touch docs/docs/module-1/new-topic.md
```

### 2. Follow the Chapter Template
Create your chapter using this template:

```markdown
---
title: Your Chapter Title
description: Brief description of what this chapter covers
tags: [tag1, tag2, tag3]
---

# Your Chapter Title

## Learning Objectives
- Objective 1
- Objective 2
- Objective 3

## Introduction
[Your content here]

## Main Content
[Your main content sections]

## Summary
[Key takeaways from the chapter]

## Further Reading
[Links to additional resources]
```

### 3. Update Navigation
After creating a new chapter, update `sidebars.js` to add it to the navigation:

```javascript
// Example sidebar configuration
module.exports = {
  docs: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/introduction',
        'module-1/ros2-middleware',
        // Add your new chapter here:
        'module-1/new-topic',
      ],
    },
    // ... other modules
  ],
};
```

## Using Claude Code for Content Generation

### 1. Prepare Specifications
Before generating content, ensure you have clear specifications in the Spec-Kit Plus format:
- Define learning objectives
- Outline key concepts to cover
- Specify technical accuracy requirements

### 2. Generate Content
Use Claude Code to generate technical content that aligns with real-world robotics concepts:
- Focus on the Physical AI orientation principle
- Ensure content relates to humanoid robotics
- Maintain clarity for technical learners

## Building and Deployment

### Build for Production
```bash
npm run build
```

### Serve Built Site Locally
```bash
npm run serve
```

### Deployment to GitHub Pages
The site is automatically deployed to GitHub Pages when changes are pushed to the main branch.

## Best Practices

### Content Creation
- Follow the constitution principles (Spec-Driven Development, AI-Assisted Accuracy, etc.)
- Ensure content is reproducible and verifiable
- Maintain consistency with Docusaurus guidelines
- Focus on the connection between AI concepts and physical robotic systems

### Technical Requirements
- Write content in Markdown format
- Use proper headings hierarchy (H1 for title, H2-H4 for sections)
- Include code examples with proper syntax highlighting
- Add alt text to images for accessibility

### Quality Assurance
- Verify all technical explanations align with real-world robotics concepts
- Test all code examples and ensure they are conceptually correct
- Ensure chapters relate to Physical AI or humanoid robotics
- Review content for clarity and technical accuracy