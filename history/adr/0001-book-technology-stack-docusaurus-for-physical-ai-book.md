# ADR-0001: Book Technology Stack: Docusaurus for Physical AI Book

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-17
- **Feature:** 001-physical-ai-book
- **Context:** Need to select a technology stack for publishing a comprehensive technical book on Physical AI and Humanoid Robotics. The solution must support Markdown-based content creation, educational features, proper navigation for learning modules, and deployment to GitHub Pages. The book will contain 10+ chapters organized in modules covering ROS 2, Gazebo, Unity, and Physical AI concepts, targeting AI students and developers.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

- Framework: Docusaurus 3.x as the static site generator for documentation
- Content Format: Markdown files for all book chapters and content
- Deployment: GitHub Pages for hosting the published book
- Customization: Docusaurus themes and components for educational features
- Build System: Node.js/npm for dependency management and build process
- Search: Built-in Docusaurus search functionality
- Navigation: Hierarchical sidebar navigation organized by course modules

## Consequences

### Positive

- Markdown-based content creation aligns with the requirement for Claude Code-generated content
- Docusaurus provides excellent support for documentation sites with built-in features like search, versioning, and responsive design
- GitHub Pages deployment is free, reliable, and integrates well with Git workflow
- Docusaurus has strong support for organizing content in structured documentation sites
- The framework supports custom components for educational features like knowledge checks and code examples
- Responsive design ensures accessibility across multiple devices for students
- Built-in SEO features help with discoverability of the educational content

### Negative

- Lock-in to Docusaurus framework may limit flexibility for future customizations
- Requires Node.js environment for local development and builds
- Potential performance limitations for very large documentation sites
- Learning curve for team members unfamiliar with Docusaurus
- Customization beyond Docusaurus capabilities may require significant effort
- Dependency on npm ecosystem and potential security vulnerabilities in packages

## Alternatives Considered

Alternative Stack A: Custom React application with MDX
- Framework: Next.js + MDX for Markdown rendering
- Deployment: Vercel or Netlify
- Custom components for educational features
- Why rejected: More complex setup, requires more development time, less built-in documentation features

Alternative Stack B: Static site generators (Jekyll, Hugo)
- Framework: Jekyll (Ruby-based) or Hugo (Go-based)
- Deployment: GitHub Pages
- Why rejected: Less Markdown flexibility for technical content, steeper learning curve for team, fewer built-in features for documentation sites

Alternative Stack C: GitBook
- Framework: GitBook platform
- Deployment: GitBook hosting
- Built-in educational features
- Why rejected: Less customization control, potential vendor lock-in to GitBook platform, less flexibility for custom educational components

## References

- Feature Spec: specs/001-physical-ai-book/spec.md
- Implementation Plan: specs/001-physical-ai-book/plan.md
- Related ADRs: None
- Evaluator Evidence: specs/001-physical-ai-book/research.md
