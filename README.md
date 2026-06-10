# Agentic AI for ROV Control

**Multi-Agent Autonomous Underwater Vehicle System — MATLAB Implementation**

> Prof. Cheng Siong Chin · Chair Professor of Intelligent Systems Modelling and Simulation  
> Newcastle University–NVIDIA Joint Laboratory · Newcastle University Singapore

---

![Simulation Dashboard](rov_agentic_simulation_v2.png)

*Multi-agent simulation dashboard: 3D mission trajectory (depth-coded), obstacle field, Mission Agent waypoint progression, Sonar Agent proximity, Navigation Agent thruster commands with fault window, and Fault Agent depth overlay.*

---

## Overview

This repository implements a complete **agentic AI architecture** for the autonomous control of Remotely Operated Vehicles (ROVs) in subsea inspection missions. Five specialised AI agents collaborate in real time at 20 Hz to plan, navigate, sense, self-monitor, and communicate — executing a full eight-waypoint inspection survey with mid-mission fault detection and recovery, without human intervention.

The system moves beyond conventional single-controller autopilots toward a **supervisory autonomy** model where each agent owns a distinct perception–decision–action loop, shares state through a structured interface, and degrades gracefully under actuator faults.

---

## Key Features

- **Five-agent architecture** — Mission Planning, Navigation, Sonar, Fault Monitoring, and Communication agents operating in a coordinated 20 Hz control loop
- **PD control + artificial potential fields** — smooth waypoint tracking with sonar-triggered obstacle repulsion
- **Mid-mission fault recovery** — automatic detection of thruster degradation with safe-mode thrust scaling and continued mission progress
- **Realistic sensor modelling** — 16-beam multibeam sonar with Gaussian noise, acoustic modem with latency jitter and 5% packet loss
- **Simplified 6-DOF dynamics** — Euler integration with quadratic hydrodynamic drag and body-to-world frame rotation
- **Live 3-D animation** — multi-panel MATLAB figure rendering trajectory, ROV mesh, thruster arrows, and four telemetry subplots at ~25 fps

---

## System Architecture

```
rov_agentic_main.m
│
├── create_environment.m     →  obstacles, seabed, current
├── create_mission.m         →  8-waypoint inspection route + task list
│
└── Main control loop (20 Hz, T = 60 s)
    │
    ├── SonarAgent.m         →  16-beam sonar, obstacle range + direction
    ├── MissionAgent.m       →  waypoint FSM, dwell timer, task dispatch
    ├── NavAgent.m           →  PD law + potential field → [Fx Fy Fz Mz]
    ├── FaultAgent.m         →  health monitor, fault inject, safe-mode
    ├── CommAgent.m          →  acoustic modem uplink simulation
    │
    ├── rov_dynamics.m       →  6-DOF Euler integration
    └── animate_rov.m        →  live 3-D telemetry animation
```

### Agent Roles

| Agent | Responsibility | Key Parameters |
|---|---|---|
| **Mission Planning** | Waypoint sequencing, task dwell, state machine | 8 waypoints, 2.5 s dwell, 2.0 m tolerance |
| **Navigation** | PD position control, obstacle avoidance | Kp=[2.5 2.5 3.0], Kd=[1.5 1.5 2.0], rep_gain=40 |
| **Sonar** | Multibeam obstacle sensing, noise model | 16 beams, range 20 m, σ=0.2 m noise |
| **Fault Monitoring** | Thruster health, velocity limits, anomaly log | Fault inject t=25–33 s, vel_limit=3 m/s |
| **Communication** | Acoustic modem uplink, packet loss model | 300 ms ± 100 ms latency, PDR ≈ 95% |

---

## Repository Structure

```
rov_agentic/
├── README.md
├── rov_agentic_main.m          # Entry point — run this
├── rov_dynamics.m              # 6-DOF simplified dynamics
├── NavAgent.m                  # Navigation agent (PD + APF)
├── MissionAgent.m              # Mission planning agent (FSM)
├── SonarAgent.m                # Sonar sensing agent
├── FaultAgent.m                # Fault monitoring agent
├── CommAgent.m                 # Communication agent
├── animate_rov.m               # Live 3-D animation
├── create_environment.m        # Scene definition
└── create_mission.m            # Mission definition
```

---

## Requirements

| Requirement | Version |
|---|---|
| MATLAB | R2021a or later |
| Toolboxes | None required (pure MATLAB) |
| Operating System | Windows / macOS / Linux |

No additional toolboxes are required. The system uses only core MATLAB functions and object-oriented classes.

---

## Getting Started

**1. Clone the repository**

```bash
git clone https://github.com/cschin/rov-agentic-ai.git
cd rov-agentic-ai
```

**2. Open MATLAB and navigate to the repository folder**

```matlab
cd('path/to/rov-agentic-ai')
```

**3. Run the main script**

```matlab
rov_agentic_main
```

The simulation runs for 60 seconds (1,201 timesteps at dt = 0.05 s), prints agent status logs to the command window, then launches the live 3-D animation automatically.

**Expected console output:**

```
[Environment] Loaded: 6 obstacles | depth=25m
[Mission] Loaded 8-waypoint inspection survey.
[NavAgent] Initialised with 8 waypoints.
[FaultAgent] Initialised. Fault injection at t=25s.
[CommAgent] Acoustic modem initialised. Latency: 300ms±100ms | Loss: 5%
[SonarAgent] Initialised with 6 obstacles, 16 beams.
[MissionAgent] Mission loaded: 8 waypoints.
[SYSTEM] Agentic ROV simulation starting...
[SYSTEM] 8 waypoints | 6 obstacles | dt=0.050s
...
[SYSTEM] Simulation complete. Rendering animation...
```

---

## Simulation Results

Running the 60-second mission across the eight-waypoint inspection route with six obstacles:

| Metric | Result |
|---|---|
| Waypoints completed | All 8 (within mission window) |
| Fault detection latency | < 1 timestep (0.05 s) |
| Safe-mode recovery | Automatic, no mission abort |
| Minimum obstacle clearance | > 1.8 m throughout |
| Communication PDR | ~95% |
| Max thrust commanded | 25 N (saturated, by design) |

---

## Simulation Outputs

### 3D Mission Trajectory

The main animation panel shows:
- **Depth-coded trajectory** — cool colours (blue/purple) at shallow depth, warm (cyan) at depth
- **ROV mesh** — orange hull pods, black structural frame, four corner thrusters, forward camera dome
- **Thruster force arrows** — red (Fx surge), green (Fy sway), blue (Fz heave)
- **Obstacle spheres** — six colour-coded subsea features
- **Waypoint markers** — green squares with WP labels

### Telemetry Panels

| Panel | Content |
|---|---|
| Mission Agent | Step-wise waypoint index progression over time |
| Sonar Agent | Nearest obstacle distance with 5 m avoidance threshold |
| Navigation Agent | Three-axis thrust history with fault window highlighted |
| Fault Agent + Depth | Binary fault flag overlaid on vehicle depth profile |

---

## Vehicle Parameters

| Parameter | Value |
|---|---|
| Mass | 12.0 kg |
| Yaw inertia | 0.8 kg·m² |
| Translational drag Cd | [8, 8, 12] N·s²/m² |
| Max thrust | 25 N per axis |
| Timestep dt | 0.05 s (20 Hz) |
| Mission duration | 60 s |

---

## Extending the System

**Add an LLM-based replanning agent**  
Replace `MissionAgent.m` with an API call to a language model for natural language mission briefs and dynamic goal revision.

**Multi-ROV coordination**  
Extend `CommAgent.m` to a broadcast network with conflict-free task allocation across a swarm of vehicles.

**Sensor fusion**  
Add a DVL (Doppler Velocity Log) and USBL positioning model to `SonarAgent.m` for EKF-based state estimation under navigation uncertainty.

**Digital twin integration**  
Connect to NVIDIA Isaac Sim or Omniverse for photorealistic rendering and sim-to-real transfer of controller gains.

**ROS2 deployment**  
Each agent class maps directly to a ROS2 node. Compile agent classes with MATLAB Coder for embedded deployment on the ROV's onboard CPU.

---

## Flowchart

The system architecture and per-timestep agent decision cycle:

![Flowchart](docs/rov_agentic_flowchart.png)

*Five-agent control loop: Sonar → Mission → Navigation → Fault → Communication → Dynamics → repeat. The fault branch and loop-back arrow are shown with distinct connector styles.*

---

## Citation

If you use this work in your research, please cite:

```bibtex
@software{chin2025rov,
  author    = {Chin, Cheng Siong},
  title     = {Agentic AI for ROV Control: A Multi-Agent MATLAB System
               for Autonomous Underwater Inspection},
  year      = {2025},
  publisher = {GitHub},
  url       = {https://github.com/cschin/rov-agentic-ai},
  note      = {Newcastle University--NVIDIA Joint Laboratory,
               Newcastle University Singapore}
}
```

---

## Related Publication

> C. S. Chin, "Agentic AI for ROV Control: A Multi-Agent MATLAB System for Autonomous Underwater Inspection," *Medium*, 2025.  
> Available: [medium.com/@cschin](https://medium.com/@cschin)

---

## Licence

MIT Licence. See `LICENSE` for details.

---

## Contact

**Prof. Cheng Siong Chin**  
Chair Professor of Intelligent Systems Modelling and Simulation  
Director, Newcastle University–NVIDIA Joint Laboratory  
Newcastle University Singapore  

- GitHub: [@cschin](https://github.com/cschin)
- n8n creator profile: [n8n.io/creators/cschin](https://n8n.io/creators/cschin)

---

*Newcastle University–NVIDIA Joint Laboratory · Autonomous Underwater Systems Programme · Singapore*
