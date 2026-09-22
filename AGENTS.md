# Agent Guidelines & Rules

## AI Coding Principles & Workflow Rules
- **Rule 1 (Explainability)**: Never merge or accept agent code you cannot explain — read and understand it before it goes into your project (ask the AI to explain it if needed).
- **Rule 2 (Core Logic vs Boilerplate)**: Write the core logic yourself for the first implementation (e.g. the PID loop and filter on the self-balancing robot); delegate boilerplate (config, CMake, plotting scripts, ROS 2 node scaffolding, driver wiring) to AI. *(Note: Writing core logic manually applies to the first time it is written).*
- **Rule 3 (Deliberate Practice)**: Dedicate a few hours a week of unassisted Python/C++ — hand-write one project feature or do small exercises (e.g. Exercism).

## Git & Workspace
- **Clean Git History**: Create atomic, well-scoped commits with clear, descriptive commit messages. Never commit broken code or leftover scratch files.
- **Workspace Boundary**: Strictly limit file edits and operations to the current workspace directory (`/home/pi`).

## Raspberry Pi SSH Access
- ssh pi@192.168.2.2
- **Password**: `admin`

## Code Quality & Architecture
- **MVP First**: Produce the minimum viable product that satisfies requirements. Avoid premature abstraction, unnecessary boilerplate, or unrequested features.
- **Modularity & Readability**: Keep functions and components small, focused, and idiomatic. Adhere to standard linting and formatting conventions for the language in use.