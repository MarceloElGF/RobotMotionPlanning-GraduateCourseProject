# RobotMotionPlanning-GraduateCourseProject (Probabilistic Roadmap)
A duo group project of Robotic Motion Planning course of graduate studies Cal Poly Pomona.

This repository contains a Python implementation of a robot motion planning system using a Probabilistic Roadmap (PRM) algorithm. The project simulates a robot navigating through a 2D environment with obstacles to reach a target destination.

<img width="564" height="675" alt="image" src="https://github.com/user-attachments/assets/b98237fd-7a17-4ea8-99e0-e9949bb96ada" />
The image shows the obstacles, path built by the PRM planner and the robot that is about to start the movement.

🚀 Features
* **PRM Path Planning:** Generates a roadmap of feasible paths by sampling the configuration space.
* **Dynamic GUI:** Real-time visualization of the robot's movement and pathfinding logic.
* **Custom Environment:** Support for obstacle-heavy maps and coordinate-based navigation.

## 🛠️ File Overview
* `prm_planner.py`: The core algorithm for roadmap generation and node sampling.
* `path_planner.py`: High-level logic for finding the shortest path through the generated roadmap.
* `E160_gui.py`: The main entry point to launch the graphical user interface.
* `controller.py`: Manages the robot's velocity and heading to follow the planned path.
* `bsTree.py` & `Cell.py`: Supporting data structures for environment partitioning.

## 💻 How to Run
1. **Prerequisites:** Ensure you have Python installed. You may need to install dependencies:
   ```bash
   pip install numpy

## 👥 Contributors
* **Marcelo Manyari** - Path planner and PID Controller
* **Luke Gonzales** - GUI and PRM implementation
