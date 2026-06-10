classdef NavAgent < handle
%NAVAGENT  Navigation Agent — PD waypoint following with obstacle avoidance
%  Uses Proportional-Derivative control on position error with
%  artificial potential field obstacle repulsion.

    properties
        waypoints
        wp_idx          = 1
        wp_tol          = 1.5       % waypoint reach tolerance [m]
        Kp              = [2.5 2.5 3.0]   % proportional gains
        Kd              = [1.5 1.5 2.0]   % derivative gains
        F_max           = 25        % max thrust [N]
        obs_rep_gain    = 40        % obstacle repulsion gain
        obs_rep_dist    = 5.0       % repulsion activation distance [m]
        prev_err        = [0 0 0]
    end

    methods
        function obj = NavAgent(waypoints)
            obj.waypoints = waypoints;
            obj.wp_idx = 1;
            fprintf('[NavAgent] Initialised with %d waypoints.\n', size(waypoints,1));
        end

        function [thrust, status] = compute_thrust(obj, pos, vel, target, ...
                                                    obs_dist, obs_dir, dt)
            % ── Position error ──────────────────────────────
            err = target - pos;
            derr = (err - obj.prev_err) / dt;
            obj.prev_err = err;

            % ── PD control law ──────────────────────────────
            Fx = obj.Kp(1)*err(1) + obj.Kd(1)*derr(1) - vel(1)*2;
            Fy = obj.Kp(2)*err(2) + obj.Kd(2)*derr(2) - vel(2)*2;
            Fz = obj.Kp(3)*err(3) + obj.Kd(3)*derr(3) - vel(3)*2;

            % ── Obstacle avoidance (potential field) ────────
            if obs_dist < obj.obs_rep_dist && obs_dist > 0.1
                rep_mag = obj.obs_rep_gain * (1/obs_dist - 1/obj.obs_rep_dist) ...
                          / (obs_dist^2);
                Fx = Fx - rep_mag * obs_dir(1);
                Fy = Fy - rep_mag * obs_dir(2);
                Fz = Fz - rep_mag * obs_dir(3);
            end

            % ── Yaw towards target ──────────────────────────
            Mz = atan2(err(2), err(1)) * 0.5;

            % ── Saturation ───────────────────────────────────
            Fx = max(-obj.F_max, min(obj.F_max, Fx));
            Fy = max(-obj.F_max, min(obj.F_max, Fy));
            Fz = max(-obj.F_max, min(obj.F_max, Fz));
            Mz = max(-2, min(2, Mz));

            thrust = [Fx Fy Fz Mz];
            dist   = norm(err);
            status = sprintf('wp%d | err=%.1fm | Fx=%.1f Fy=%.1f Fz=%.1f', ...
                             obj.wp_idx, dist, Fx, Fy, Fz);
        end
    end
end
