# Data Model: Physical AI & Humanoid Robotics Book

## Overview
This document defines the conceptual data model for the Physical AI & Humanoid Robotics book project. Since this is primarily a documentation project, the "data" consists of content organization and metadata.

## Content Entities

### Book Chapter
**Description**: A single chapter of the book containing educational content
- **Fields**:
  - `id`: Unique identifier for the chapter
  - `title`: Display title of the chapter
  - `slug`: URL-friendly identifier
  - `module`: Module identifier (e.g., "module-1", "module-2")
  - `order`: Chapter's position within the module
  - `content`: Markdown content of the chapter
  - `prerequisites`: List of required knowledge/previous chapters
  - `learningObjectives`: List of learning objectives for the chapter
  - `tags`: Technical tags for search and categorization
  - `lastModified`: Timestamp of last content update
  - `authors`: List of contributors to the chapter

### Course Module
**Description**: A grouping of related chapters covering a specific topic area
- **Fields**:
  - `id`: Module identifier (e.g., "module-1", "module-2")
  - `title`: Module title
  - `description`: Brief description of the module
  - `chapters`: List of chapter IDs belonging to this module
  - `estimatedDuration`: Estimated time to complete the module
  - `prerequisites`: Knowledge required before starting this module
  - `learningOutcomes`: List of outcomes after completing the module

### Navigation Item
**Description**: An item in the book's navigation structure
- **Fields**:
  - `id`: Unique identifier
  - `title`: Display title
  - `type`: Type of navigation item (chapter, category, link)
  - `path`: Path to the content
  - `parentId`: ID of parent navigation item (for hierarchical structure)
  - `order`: Position in the navigation hierarchy
  - `isVisible`: Whether to show in navigation

### Code Example
**Description**: Code snippets and examples within chapters
- **Fields**:
  - `id`: Unique identifier
  - `title`: Brief description of the example
  - `language`: Programming language or format
  - `code`: The actual code content
  - `chapterId`: ID of the chapter containing this example
  - `purpose`: Explanation of what the example demonstrates
  - `complexity`: Difficulty level (beginner, intermediate, advanced)

## Relationships

### Module-Chapter Relationship
- One `Course Module` contains many `Book Chapter` entities
- Each `Book Chapter` belongs to exactly one `Course Module`

### Chapter-Code Example Relationship
- One `Book Chapter` contains many `Code Example` entities
- Each `Code Example` belongs to exactly one `Book Chapter`

### Navigation Hierarchy
- `Navigation Item` entities form a tree structure
- Each item can have zero or more child items
- Items can have parent-child relationships to create the navigation hierarchy

## Validation Rules

### Book Chapter Validation
- `title` must not be empty
- `slug` must be unique within the book
- `module` must reference an existing module
- `order` must be a positive integer
- `content` must follow Markdown format
- `learningObjectives` must contain at least one objective

### Course Module Validation
- `id` must be unique
- `title` must not be empty
- `chapters` list must not be empty
- `estimatedDuration` must be a positive number

### Navigation Item Validation
- `title` must not be empty
- `path` must be valid for Docusaurus routing
- `order` must be a non-negative integer
- `type` must be one of: "chapter", "category", "link"

## State Transitions (for content workflow)

### Chapter States
- `draft`: Content is being created
- `review`: Content is under review
- `approved`: Content has been approved
- `published`: Content is live in the book

### Transition Rules
- `draft` → `review`: When content is ready for review
- `review` → `draft`: When changes are requested
- `review` → `approved`: When content passes review
- `approved` → `published`: When content is ready for publication

## Metadata Requirements

### SEO and Accessibility
Each chapter should include:
- Title for search engines
- Description for search result snippets
- Keywords for searchability
- Accessibility attributes for screen readers

### Learning Analytics
Each chapter should track:
- Estimated reading time
- Difficulty level
- Required prior knowledge
- Learning outcomes achieved