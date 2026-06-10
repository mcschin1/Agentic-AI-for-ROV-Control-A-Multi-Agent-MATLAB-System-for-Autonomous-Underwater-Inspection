%% =========================================================
%  Agentic AI for ROV Control  — Main Entry Point
%  Prof. Cheng Siong Chin | Newcastle University Singapore
%  =========================================================
%  System:  Multi-agent architecture with:
%           - Mission Planning Agent
%           - Navigation Agent  (PD controller)
%           - Obstacle Detection Agent (sonar simulation)
%           - Fault Monitoring Agent
%           - Communication Agent
%  Runs a 3-D simulation with live animation.
% ==========================================================
clear; clc; close all;

%% ── Global simulation parameters ────────────────────────
dt        = 0.05;          % time step [s]
T_end     = 60;            % mission duration [s]
t         = 0:dt:T_end;
N         = length(t);

%% ── Environment & Mission ────────────────────────────────
env = create_environment();
mission = create_mission(env);

%% ── ROV initial state  [x y z roll pitch yaw vx vy vz] ──
state = zeros(N, 9);
state(1,:) = [mission.waypoints(1,:), 0, 0, 0, 0, 0, 0];

%% ── Pre-allocate log arrays ──────────────────────────────
thrust_log    = zeros(N, 4);   % [Fx Fy Fz Mz]
agent_status  = cell(N, 5);    % status strings
fault_log     = zeros(N, 1);   % fault flag
sonar_log     = zeros(N, 1);   % min obstacle distance

%% ── Initialise agents ────────────────────────────────────
nav_agent     = NavAgent(mission.waypoints);
fault_agent   = FaultAgent();
comm_agent    = CommAgent();
sonar_agent   = SonarAgent(env.obstacles);
mission_agent = MissionAgent(mission.waypoints, mission.tasks);

fprintf('[SYSTEM] Agentic ROV simulation starting...\n');
fprintf('[SYSTEM] %d waypoints | %d obstacles | dt=%.3fs\n', ...
        size(mission.waypoints,1), size(env.obstacles,1), dt);

%% ═══════════════════════════════════════════════════════
%  MAIN SIMULATION LOOP
% ═══════════════════════════════════════════════════════
for k = 1:N-1
    pos = state(k,1:3);
    vel = state(k,7:9);

    %-- 1. Sonar / Obstacle Detection Agent ----------------
    [min_dist, obs_dir] = sonar_agent.sense(pos, env.obstacles);
    sonar_log(k) = min_dist;

    %-- 2. Mission Planning Agent --------------------------
    [wp_target, task_str, wp_reached] = mission_agent.update(pos, dt);

    %-- 3. Navigation Agent --------------------------------
    [thrust, nav_str] = nav_agent.compute_thrust(pos, vel, wp_target, ...
                                                  min_dist, obs_dir, dt);

    %-- 4. Fault Monitoring Agent --------------------------
    [fault, fault_str] = fault_agent.check(thrust, vel, t(k));
    if fault
        thrust = fault_agent.safe_thrust(thrust);
    end
    fault_log(k) = fault;

    %-- 5. Communication Agent ----------------------------
    comm_str = comm_agent.broadcast(pos, wp_target, nav_str, fault_str, t(k));

    %-- 6. Dynamics integration (6-DOF simplified) --------
    state(k+1,:) = rov_dynamics(state(k,:), thrust, dt);

    %-- Log -----------------------------------------------
    thrust_log(k,:)   = thrust;
    agent_status{k,1} = task_str;
    agent_status{k,2} = nav_str;
    agent_status{k,3} = fault_str;
    agent_status{k,4} = comm_str;
    agent_status{k,5} = sprintf('sonar=%.1fm', min_dist);
end

fprintf('[SYSTEM] Simulation complete. Rendering animation...\n');

%% ── Render animated 3-D visualisation ───────────────────
animate_rov(t, state, mission, env, thrust_log, sonar_log, fault_log);

fprintf('[SYSTEM] Animation complete.\n');
