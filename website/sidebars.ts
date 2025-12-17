import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro', 'physical-ai-overview'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/intro',
        'module-1/ros2-middleware',
        'module-1/nodes-topics-services',
        'module-1/python-ai-bridging',
        'module-1/urdf-structure',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/intro',
        'module-2/physics-simulation',
        'module-2/sensor-simulation',
        'module-2/digital-twins',
        'module-2/unity-visualization',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Additional Topics',
      items: [
        'additional/physical-ai-intro',
        'additional/ai-physical-world',
        'additional/humanoid-robotics',
      ],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Assessment & Exercises',
      items: [
        'assessment/knowledge-checks',
        'assessment/practical-exercises',
      ],
      collapsed: true,
    },
  ],
};

export default sidebars;
