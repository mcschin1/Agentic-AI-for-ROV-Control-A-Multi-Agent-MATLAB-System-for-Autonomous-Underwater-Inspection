classdef FaultAgent < handle
%FAULTAGENT  Monitors thruster health, velocity anomalies, and depth limits
%  Injects simulated thruster degradation for realism.

    properties
        thrust_limit   = 22       % healthy max thrust [N]
        vel_limit      = 3.0      % max allowable velocity [m/s]
        fault_active   = false
        fault_type     = 'none'
        fault_start    = -1
        fault_duration = 8        % duration of injected fault [s]
        fault_trigger  = 25       % time at which fault is injected [s]
        log_count      = 0
    end

    methods
        function obj = FaultAgent()
            fprintf('[FaultAgent] Initialised. Fault injection at t=%.0fs.\n', ...
                    obj.fault_trigger);
        end

        function [fault_flag, status] = check(obj, thrust, vel, t)
            fault_flag = false;
            obj.fault_active = false;

            % Inject simulated thruster degradation
            if t >= obj.fault_trigger && t < obj.fault_trigger + obj.fault_duration
                if obj.fault_start < 0; obj.fault_start = t; end
                obj.fault_active = true;
                obj.fault_type = 'Thruster-1 degraded (50%)';
                fault_flag = true;
            else
                obj.fault_type = 'none';
                obj.fault_start = -1;
            end

            % Over-speed check
            if norm(vel) > obj.vel_limit
                fault_flag = true;
                obj.fault_type = [obj.fault_type ' | OVERSPEED'];
            end

            % Thrust saturation check
            if max(abs(thrust)) > obj.thrust_limit
                fault_flag = true;
                obj.fault_type = [obj.fault_type ' | THRUST_SAT'];
            end

            if fault_flag
                obj.log_count = obj.log_count + 1;
                status = sprintf('⚠ FAULT: %s', obj.fault_type);
            else
                status = 'OK — all systems nominal';
            end
        end

        function t_safe = safe_thrust(obj, thrust)
            % Degrade thrust on primary thruster during fault
            t_safe = thrust;
            t_safe(1) = t_safe(1) * 0.5;
            t_safe = max(-20, min(20, t_safe));
        end
    end
end
