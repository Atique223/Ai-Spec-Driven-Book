# Physical AI & Humanoid Robotics Book

This website contains a comprehensive educational resource on Physical AI and Humanoid Robotics, built with [Docusaurus](https://docusaurus.io/).

## About This Book

This book explores the fascinating intersection of artificial intelligence and physical robotic systems, bridging the gap between digital intelligence and real-world applications. It covers:

- **Module 1**: The Robotic Nervous System (ROS 2) - Understanding middleware, communication patterns, and control systems
- **Module 2**: The Digital Twin (Gazebo & Unity) - Exploring simulation environments for safe development
- **Additional Topics**: Physical AI concepts, human-robot interaction, and humanoid robotics

## Target Audience

This book is designed for undergraduate-level students and professionals with basic programming knowledge and fundamental AI concepts, suitable for junior to senior undergraduate level courses.

## Features

- Comprehensive coverage of ROS 2 fundamentals
- Physics simulation with Gazebo
- Unity integration for visualization
- Sensor simulation and digital twin concepts
- Practical exercises and knowledge checks
- Humanoid robotics applications

## Installation

```bash
# Install dependencies
npm install
```

## Local Development

```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Building for Production

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true npm run deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> npm run deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.

## Contributing

This book was created using Spec-Kit Plus methodology with Claude Code for content generation, following spec-driven development principles. All content should align with the project constitution and maintain consistency with Physical AI and Humanoid Robotics educational goals.

## License

This educational content is provided for learning purposes in the field of Physical AI and Humanoid Robotics.
