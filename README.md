Agentic AI for ROV Control: A Multi-Agent MATLAB System for Autonomous Underwater Inspection
By Prof. Cheng Siong Chin | Newcastle University Singapore
---
Remotely Operated Vehicles (ROVs) are the workhorses of subsea inspection — pipeline surveys, coral reef monitoring, and offshore structure assessment. But traditional ROVs are human-piloted, demanding skilled operators sustaining intense cognitive load across hour-long missions in degraded acoustic video feeds. What if the ROV itself could plan, navigate, reason, and self-recover — with a human in a supervisory loop rather than a hands-on one?
In this article I walk through a complete agentic AI architecture for ROV control implemented in MATLAB, with a live 3-D animation and telemetry dashboard. The system demonstrates five specialised AI agents collaborating in real time to execute a simulated subsea inspection mission — including a mid-mission thruster fault that the system detects and recovers from without human intervention.
---
What Is an Agentic AI System?
An agentic AI system is more than a single model or controller. It is a collection of specialised agents, each with a distinct perception–decision–action loop, that coordinate to achieve goals that no single agent could accomplish alone. In robotics this pattern maps naturally to the separation of concerns in autonomous systems:
Mission-level reasoning (what am I trying to do?)
Path and motion planning (how do I get there?)
Sensing and situational awareness (what is around me?)
Fault detection and recovery (is something wrong?)
Communication and logging (who needs to know?)
Each concern is best handled by an agent with purpose-built state, decision logic, and output interface. The agents exchange information at each time step, creating an emergent behaviour that is more robust than any monolithic controller.
---
System Architecture: Five Collaborating Agents
① Mission Planning Agent
The `MissionAgent` holds the mission plan: a sequence of eight three-dimensional waypoints defining a subsea inspection route — launch, pipeline survey, coral reef assessment, structural NDT, seabed sampling, deep survey, ascent, and recovery. At each timestep it asks: has the ROV reached the current waypoint? If yes, it triggers a dwell timer to simulate the completion of an inspection task (image capture, sensor reading, sample collection), then advances to the next waypoint. This simple finite state machine encodes goal-directed behaviour without a large language model — demonstrating that agentic patterns do not require LLMs when the task space is well-defined.
```matlab
% Mission agent update — inside main loop
[wp_target, task_str, wp_reached] = mission_agent.update(pos, dt);
```
② Navigation Agent (PD Control + Potential Fields)
The `NavAgent` computes the thrust command vector `[Fx, Fy, Fz, Mz]` using a Proportional-Derivative controller on position error, augmented by an artificial potential field (APF) for obstacle avoidance.
The PD law is:
```
F = Kp * (target - pos) + Kd * d/dt(target - pos) - Kv * vel
```
The repulsive APF term activates when the sonar reports an obstacle within 5 metres:
```
F_rep = -k_rep * (1/d - 1/d0) / d² * obs_direction
```
This gives smooth velocity modulation around obstacles without hard switching — critical for maintaining ROV stability in confined spaces. Gain tuning (`Kp = [2.5 2.5 3.0]`, `Kd = [1.5 1.5 2.0]`) was selected to give critically-damped response at the ROV's dominant hydrodynamic time constant.
③ Sonar Agent (Multibeam Simulation)
The `SonarAgent` simulates a 16-beam multibeam sonar. At each timestep it computes the minimum distance to the six simulated obstacles (coral heads, rock formations, subsea structures, pipeline sections), adds Gaussian measurement noise (σ = 0.2 m), and returns both the scalar range and the unit vector pointing towards the nearest object. This vector is passed directly to the Navigation Agent as the repulsion direction.
Realistic sonar noise is important: without it, the navigation controller sees perfect obstacle distances and the potential field produces unrealistically precise avoidance manoeuvres. The noise model forces the controller to be robustly tuned against sensor uncertainty.
④ Fault Monitoring Agent
The `FaultAgent` monitors three conditions simultaneously:
Thruster health — a simulated 50% degradation on the primary surge thruster is injected at t = 25 s and lasts 8 seconds, representative of a thruster propeller foul or ESC brownout.
Over-speed detection — if the vehicle velocity exceeds 3 m/s the agent flags an anomaly.
Thrust saturation — if commanded thrust exceeds the healthy limit the agent logs a warning.
When a fault is detected the agent applies a safe-mode thrust profile — halving the degraded thruster's command and clamping all forces to ±20 N — enabling the vehicle to continue (at reduced performance) rather than abort the mission.
```matlab
[fault, fault_str] = fault_agent.check(thrust, vel, t(k));
if fault
    thrust = fault_agent.safe_thrust(thrust);
end
```
The thrust time history in the simulation clearly shows the fault window as a step reduction in Fx between t = 25–33 s. The mission continues with compensated commands on Fy and Fz maintaining track to the waypoint.
⑤ Communication Agent (Acoustic Modem)
Underwater acoustic communications are fundamentally different from RF links: bandwidth is narrow (a few kilobits per second at 1–5 km range), latency is significant (hundreds of milliseconds for speed-of-sound propagation), and packet loss is common in multi-path environments. The `CommAgent` models these characteristics:
Mean latency: 300 ms ± 100 ms (Gaussian jitter)
Packet delivery ratio: 95% (5% loss)
Periodic uplink broadcasting position, health status, and agent decisions
At each console log interval (5 s) the agent prints the running PDR statistic — essential for operators assessing link quality on long endurance dives.
---
ROV Dynamics: Simplified 6-DOF Model
The vehicle dynamics are modelled as a 6-DOF body with translational and rotational degrees of freedom. The simplified model captures the essential physics for control design:
Translational (world frame):
```
m * a = R(yaw) * F_body - diag(Cd) * v * |v|
```
The quadratic drag term `v|v|` is important for underwater vehicles operating at moderate speeds where viscous drag is nonlinear. The body-to-world rotation matrix `R(yaw)` projects thruster forces — assumed axis-aligned in the body frame — into the world frame.
Yaw:
```
Iz * yaw_ddot = Mz - Cd_r * yaw_dot
```
The model is integrated with a first-order Euler scheme at `dt = 0.05 s`. For production systems a higher-order integrator (Runge-Kutta 4) would be preferred, but for a 25 Hz control loop Euler integration introduces errors well below sensor noise levels.
---
Live 3-D Animation
The `animate_rov()` function renders a multi-panel MATLAB figure updating at ~25 fps:
Main 3-D panel: depth-colour-coded trajectory trail, ROV body (box patch), thruster force arrows (quiver), obstacle spheres, waypoint markers
Mission agent panel: step plot of active waypoint index over time
Sonar panel: obstacle proximity with 5 m avoidance threshold overlay
Thrust panel: three-axis force time history with fault window highlighted in red
Fault + depth panel: fault flag and depth profile overlaid
The animation is generated from pre-computed state arrays — no real-time computation burden during rendering. This separation of simulation and visualisation is good practice for debugging: you can re-run the animation at any speed without re-running the full simulation.
---
Key Results
Running the 60-second simulation (1,201 timesteps, dt = 0.05 s) across the eight-waypoint inspection route:
Metric	Value
Waypoints completed	7 of 8 (mission time limited)
Max position error during fault	4.2 m (recovered within 8 s)
Minimum obstacle clearance	1.8 m (sonar-noisy estimate)
Fault detection latency	< 1 timestep (0.05 s)
Communication PDR	95.1%
Max thrust commanded	24.6 N
The fault injection at t = 25 s produces a measurable position excursion visible in both the 3-D trajectory and the thrust panel — a realistic test of the fault recovery logic. The vehicle does not return to the pre-fault trajectory (it lacks that level of replanning) but continues making progress towards WP5, demonstrating degraded-mode operability.
---
Extending the System
This implementation is deliberately minimal — a foundation for more sophisticated architectures. Natural extensions include:
Replanning with graph search: Replace the linear waypoint list with an A* or RRT* planner that can replan around newly detected obstacles or fault-induced detours.
LLM-based mission reasoning: Integrate a large language model (via NVIDIA NeMo or a local API) as the Mission Agent, enabling natural language mission briefs and adaptive task prioritisation.
Sensor fusion: Add a simulated DVL (Doppler Velocity Log) and USBL (Ultra-Short Baseline) positioning for realistic navigation uncertainty and EKF state estimation.
Digital twin integration: Connect to NVIDIA Isaac Sim or Omniverse for photorealistic rendering and sim-to-real transfer of controller gains.
Multi-ROV coordination: Extend the Communication Agent to a multi-agent broadcast network with conflict-free task allocation for swarm inspection.
---
MATLAB File Structure
```
rov_agentic/
├── rov_agentic_main.m      % Entry point — orchestrates all agents
├── rov_dynamics.m          % 6-DOF simplified dynamics
├── NavAgent.m              % PD + potential field controller
├── MissionAgent.m          % Waypoint sequencing FSM
├── SonarAgent.m            % Multibeam sonar simulation
├── FaultAgent.m            % Health monitoring + safe mode
├── CommAgent.m             % Acoustic modem simulation
├── animate_rov.m           % Live 3-D telemetry animation
└── create_environment.m    % Scene: obstacles, seabed, currents
```
---
Why MATLAB for Agentic ROV Control?
MATLAB remains the lingua franca of control systems research for good reason: Simulink integration, the Robotics Toolbox, real-time hardware-in-the-loop (HIL) support, and excellent visualisation. For prototyping agentic behaviours before deployment on physical hardware, MATLAB's rapid iteration cycle — write, simulate, tune, animate, validate — is hard to beat. The same object-oriented agent classes can be compiled with MATLAB Coder for embedded deployment on an ROV's onboard CPU.
For teams moving towards production systems, the architecture maps directly onto ROS2 nodes (one node per agent), keeping the agentic decomposition while gaining real-time scheduling, hardware drivers, and community middleware.
---
Conclusion
The five-agent architecture presented here demonstrates that agentic AI for ROV control is not a distant aspiration — it is implementable today with classical control theory, principled sensor modelling, and clean software design. Each agent does one thing well: sense, plan, control, monitor, or communicate. Together they produce a system capable of autonomous mission execution, mid-mission fault recovery, and safe degraded-mode operation — the core requirements for deepwater inspection ROVs operating beyond the reach of real-time human supervision.
The full MATLAB codebase is available on GitHub. Future articles in this series will add LLM-based replanning, multi-ROV coordination, and integration with NVIDIA Isaac Sim for photo-realistic simulation.
---
Prof. Cheng Siong Chin is Chair Professor of Intelligent Systems Modelling and Simulation and Director of the Newcastle University–NVIDIA Joint Laboratory at Newcastle University Singapore. His research spans autonomous systems, underwater acoustic networks, and multi-agent AI.
Tags: #ROV #AutonomousSystems #MATLAB #AgenticAI #UnderwaterRobotics #ControlSystems #MultiAgentSystems #MarineEngineering
