# MataTurboPi – Robotics & AI Lessons Library

This repository is a **central library for learning Robotics, AI, and Python** using real robots.

It’s designed for use with Matamoe robots (TurboPi and related platforms) and supports classroom lessons, after-school programmes, and inter-school challenges. Lessons are hands-on, practical, and focused on learning by doing.

---

## What this repo is for

This repo contains:
- **Shared robot libraries** (movement, vision, sound, control)
- **Lesson templates** used by students in Jupyter
- **Tools** for calibration and setup

Students never edit the core libraries directly.  
Instead, lesson templates are copied to student workspaces so everyone starts clean and can safely experiment.

---

## Learning topics covered

### 🤖 Robot Movement
Learn how to navigate robots around obstacles with style.

Students will:
- Move forwards and backwards
- Spin and turn
- Slide sideways
- Drift using velocity control
- Combine moves into smooth sequences

---

### 🧠 AI & Python Coding
Use real-world robot tasks to make Python and AI concepts click.

Students will:
- Write Python that controls real hardware
- Use logic, loops, and functions in meaningful ways
- Connect sensors, motors, and AI together
- See instant physical feedback from their code

---

### 👁️ Computer Vision
Use robot cameras and AI to understand the world.

Students will:
- Detect coloured balls using the camera
- Track objects in real time
- Make decisions based on vision data
- Push balls into goals using autonomous behaviour

---

### 🎬 Animate the Robots
Bring robots to life with movement, sound, and personality.

Students will:
- Animate robots to move and talk
- Trigger sounds and speech
- Create short “performances”
- Make robots feel more like characters (think *Cars*, not calculators)

---

### 🏁 Robot Jam Competitions
Put skills to the test in friendly inter-school challenges.

Every two weeks:
- Students visit another school
- Robots compete in structured challenges
- Teams refine movement, AI, and strategy
- Learning is reinforced through play and iteration

---

## Repository structure

```text
common/
  lib/        # Shared robot Python libraries
  sounds/     # Shared sound assets

tools/
  # Calibration and maintenance notebooks

lessons/
  lesson_01_robot_demo/
  lesson_02_...
