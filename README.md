# Motion Control for Car & Robotic Arm  
![Python 3.8+](https://img.shields.io/badge/Python-3.8%2B-blue)  
![ROS Foxy](https://img.shields.io/badge/ROS-Foxy-green)  

A set of ROS-based controllers implementing **PID pure-pursuit** and **Model-Predictive Control (MPC)** for both a MuSHR wheeled vehicle and a WidowX250 robotic arm. Includes test suites, visualization tools, and parameter tuning writeups. :contentReference[oaicite:0]{index=0}

---

## Table of Contents

- [Overview](#overview)  
- [Features](#features)  
- [Demos & Visuals](#demos--visuals)  
- [Installation](#installation)  
- [Usage](#usage)  
- [Folder Structure](#folder-structure)  
- [Contributing](#contributing)  
- [License](#license)  
- [Acknowledgments](#acknowledgments)

---

## Overview

This repository contains Homework 5 for CS4750 Robotic Foundations, where you’ll implement:  
1. **Reference selection & error computation** for lookahead-based controllers :contentReference[oaicite:1]{index=1}  
2. **PID pure-pursuit** for path tracking on a MuSHR car and joint‐space control on WidowX250 :contentReference[oaicite:2]{index=2}  
3. **MPC** with rollout sampling, cost evaluation, and obstacle avoidance in a slalom world :contentReference[oaicite:3]{index=3}  
4. **Automated tests** in `test/` that validate each component :contentReference[oaicite:4]{index=4}

---

## Features

- 📏 **BaseController**: Reference‐state lookahead & coordinate‐frame error (`get_reference_index`, `compute_position_in_frame`) :contentReference[oaicite:5]{index=5}  
- 🔄 **PID Controller**: Implements proportional, integral, and derivative terms for cross-track error (`PID.get_control`) :contentReference[oaicite:6]{index=6}  
- 🚀 **MPC Controller**: Samples control sequences, rollouts with kinematic car model, and cost‐based action selection (`MPC.get_control`) :contentReference[oaicite:7]{index=7}  
- 🧪 **Test Suites**:  
  - `test/controller.py` for base & PID logic  
  - `test/pid.py` for PID correctness  
  - `test/mpc.py` for MPC sampling & rollout :contentReference[oaicite:8]{index=8}  
- 📊 **Visualization**:  
  - RViz config for real‐time overlay (`control.rviz`)  
  - Plots of error frames, cost landscapes, and path comparisons :contentReference[oaicite:9]{index=9}

---

## Demos & Visuals

### Car Controllers

![PID Wave Path](pid_wave.png)  
*PID tracking of a “wave” path, showing minimal cross‐track error.* :contentReference[oaicite:10]{index=10}  

![Cost Landscape](cost_plot.png)  
*MPC rollout costs, highlighting collision penalties vs. distance error.* :contentReference[oaicite:11]{index=11}  

![Slalom Obstacle Course](slalom.png)  
*MPC navigating a slalom map under ROS simulation.* :contentReference[oaicite:12]{index=12}  

### Arm Controller

<details open>  
<summary>Arm Control Block Diagram</summary>  
![Arm Control SVG](control.svg)  
*PID closed‐loop block diagram for WidowX250 joint control.* :contentReference[oaicite:13]{index=13}  
</details>

![Error in New Frame](error_new_frame.png)  
*Visualization of position‐error components in the reference frame.* :contentReference[oaicite:14]{index=14}  

---
