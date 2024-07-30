# Robotic Simulation in MATLAB

This repository contains a MATLAB script to simulate a robotic scenario in a 3D environment. The script sets up the environment, adds obstacles, plans a path for the robot, and visualizes the robot's movement through the environment.

## Overview

### Initialization

Creates a `robotScenario` object and adds a floor to the environment.

### Adding Walls

Adds both outer and inner walls to create obstacles and pathways. The walls are defined with specific dimensions and colors.

### Visualization

Displays the 3D environment with lighting and view settings.

### Binary Occupancy Map

Creates and displays a binary occupancy map to represent the obstacles in the environment.

### Path Planning

Uses a PRM planner to find a path from a starting position to a goal position, generating waypoints for the robot to follow.

### Trajectory Generation

Generates a trajectory for the robot to follow, including timing information.

### Robot Setup

Loads the Husky robot model and sets up its trajectory in the simulation environment.

### Visualization of Waypoints

Displays the environment and plots the waypoints in the 3D view.

### Running the Simulation

Runs the simulation, updating the scenario at a rate of 20 Hz, and stops when the robot reaches the goal position.

## Requirements

- MATLAB
- Robotics System Toolbox

## Usage

1. **Clone the Repository:**

   ```bash
   git clone https://github.com/yourusername/robotic-simulation-matlab.git
