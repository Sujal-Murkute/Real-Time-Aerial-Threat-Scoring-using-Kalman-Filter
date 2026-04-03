# Real-Time Aerial Threat Scoring using Kalman Filter

## Overview
A real-time aerial threat scoring framework integrating Kalman Filter-based radar tracking with a pipelined Verilog hardware accelerator.

## Features
- Kalman Filter / EKF tracking
- Stealth & evasive targets
- Formation detection
- Verilog threat scoring (0–9)
- DV-style testbench
- UI

## Tech Stack
- MATLAB
- Verilog
- ModelSim

## How to Run
### MATLAB
Run:
radar_simulation.m

### Verilog
vlog threat_scorer.v
vlog tb_threat_scorer.v
vsim tb_threat_scorer
run -all

###UI
run index.html with live server
