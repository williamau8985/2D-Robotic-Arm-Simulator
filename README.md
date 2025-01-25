# 2D Robotic Arm Simulator

This Python application is an interactive 2D robotic arm simulator built using Pygame. It allows users to explore inverse kinematics by controlling the robotic arm with their mouse. The simulator calculates joint angles to reach the target point and visually represents the arm's movement, angles, and lengths.

---

## Features

- **Interactive Simulation**: Move the mouse to control the end effector of the robotic arm.
- **Inverse Kinematics**: Real-time calculation of joint angles (shoulder and elbow) using mathematical formulas.
- **Visual Representations**:
  - Angles are drawn as arcs with degree measurements.
  - Lengths of arm segments are displayed with measurement indicators.
- **LaTeX Rendering**: Mathematical formulas for inverse kinematics are pre-rendered and displayed in the GUI.
- **Customizable Arm Parameters**: Modify arm segment lengths and base position to test different setups.

---

## Requirements

- Python 3.8+
- Required Python Libraries:
  - `pygame`
  - `matplotlib`

Install the dependencies using pip:

```bash
pip install pygame matplotlib
