<!-- SYNC IMPACT REPORT:
Version change: 1.0.0 -> 1.1.0
Modified principles: Spec-Driven Accuracy → Spec-Driven Development (renamed and expanded), AI-Augmented Clarity → AI-Assisted Accuracy (renamed and aligned with your principles), others replaced with your specific principles
Added sections: Embodied Intelligence Focus, Clarity for Technical Learners, Physical AI Orientation, Humanoid Robotics Context, Simulation-First Learning, Course Alignment Principles, Content Scope with Module details
Removed sections: Original generic principles
Templates requiring updates:
- .specify/templates/plan-template.md ✅ updated
- .specify/templates/spec-template.md ✅ updated
- .specify/templates/tasks-template.md ✅ updated
- .specify/templates/commands/*.md ⚠ pending review
- README.md ⚠ pending review
Follow-up TODOs: None
-->

# AI/Spec-Driven Book Creation Constitution: Physical AI & Humanoid Robotics

## Core Principles

### Spec-Driven Development
All book content, chapters, and structure must be driven by explicit specifications defined in Spec-Kit Plus. Every aspect of the book creation process must follow formal specifications rather than ad-hoc choices.

### AI-Assisted Accuracy
Claude Code is used to generate content, but all technical explanations must align with real-world robotics and AI concepts. AI assistance should enhance clarity and accessibility while maintaining technical accuracy and real-world applicability.

### Embodied Intelligence Focus
Emphasize AI systems operating in the physical world, bridging digital intelligence and physical robotic bodies. Content must clearly demonstrate the connection between abstract AI concepts and their physical implementations in robotics.

### Clarity for Technical Learners
Content must be understandable for students with AI and programming background, especially those learning robotics. Technical concepts should be explained with clear definitions, practical context, and real-world relevance suitable for beginner-to-intermediate robotics learners.

### Reproducibility & Verifiability
All technical workflows, examples, and explanations should be reproducible or logically traceable. Every claim, example, and demonstration must be testable and produce consistent results across environments.

## Course Alignment Principles

### Physical AI Orientation
The book must reflect AI systems that understand and interact with physical laws (motion, gravity, sensors). Content must emphasize how AI algorithms operate in physical environments with real-world constraints.

### Humanoid Robotics Context
Focus on humanoid robots operating in simulated and real environments. All examples and explanations should relate to humanoid robot design, control, and operation.

### Simulation-First Learning
Use simulation tools (Gazebo, Unity, Isaac) as a bridge to real-world deployment. Prioritize simulation-based learning experiences that prepare readers for real-world applications.

## Key Standards

All chapters must map clearly to the course modules:
- Physical AI concepts
- ROS 2 fundamentals
- Digital twins and simulation

Technical concepts must be explained with:
- Clear definitions
- Practical context
- Real-world relevance

Docusaurus must be used as the documentation/book framework.

GitHub Pages deployment must correctly render:
- Chapters
- Navigation
- Sidebar structure

Writing quality target:
- Clear technical English
- Consistent terminology
- Beginner-to-intermediate robotics level

Content Scope (Mandatory Coverage)

Module 1: The Robotic Nervous System (ROS 2)
- Role of ROS 2 as robot middleware
- Nodes, Topics, and Services
- Bridging Python AI agents to ROS controllers using rclpy
- Understanding URDF for humanoid robot structure

Module 2: The Digital Twin (Gazebo & Unity)
- Physics simulation (gravity, collisions, joints) in Gazebo
- Digital twin concepts and environment modeling
- Sensor simulation:
  - LiDAR
  - Depth Cameras
  - IMUs
- Human-robot interaction and visualization using Unity

## Constraints and Requirements

Framework: Docusaurus (Markdown-based book)

Specification driver: Spec-Kit Plus

AI writing tool: Claude Code only

Deployment target: GitHub Pages

Minimum chapters: 10

Each chapter must:
- Relate to Physical AI or humanoid robotics
- Be properly linked in navigation
- Format: Markdown source files
- Code examples (if included) must be conceptually correct and focused on learning, not production deployment

## Governance

All development and content creation must adhere to these constitutional principles. Any deviation from these principles requires explicit justification and approval. Content reviews must verify compliance with all specified standards. The constitution serves as the authoritative guide for all project decisions and trade-offs.

**Version**: 1.1.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-17