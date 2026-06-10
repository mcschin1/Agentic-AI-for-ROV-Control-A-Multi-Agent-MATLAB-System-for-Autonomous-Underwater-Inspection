classdef MissionAgent < handle
%MISSIONAGENT  Mission Planning Agent — manages waypoint sequencing and tasks

    properties
        waypoints
        tasks
        wp_idx      = 1
        wp_tol      = 2.0
        dwell_time  = 3.0       % seconds to dwell at each waypoint
        dwell_timer = 0
        dwelling    = false
        mission_complete = false
    end

    methods
        function obj = MissionAgent(waypoints, tasks)
            obj.waypoints = waypoints;
            obj.tasks = tasks;
            fprintf('[MissionAgent] Mission loaded: %d waypoints.\n', size(waypoints,1));
            for i = 1:length(tasks)
                fprintf('  WP%d → %s\n', i, tasks{i});
            end
        end

        function [target, task_str, reached] = update(obj, pos, dt)
            reached = false;
            if obj.mission_complete
                target = obj.waypoints(end,:);
                task_str = 'MISSION COMPLETE — hovering';
                return;
            end

            target = obj.waypoints(obj.wp_idx, :);
            dist   = norm(pos - target);

            if obj.dwelling
                obj.dwell_timer = obj.dwell_timer + dt;
                task_str = sprintf('[WP%d] %s — dwelling %.1fs/%.1fs', ...
                    obj.wp_idx, obj.tasks{obj.wp_idx}, ...
                    obj.dwell_timer, obj.dwell_time);
                if obj.dwell_timer >= obj.dwell_time
                    obj.dwelling = false;
                    obj.dwell_timer = 0;
                    reached = true;
                    if obj.wp_idx < size(obj.waypoints,1)
                        obj.wp_idx = obj.wp_idx + 1;
                        fprintf('[MissionAgent] Advancing to WP%d: %s\n', ...
                            obj.wp_idx, obj.tasks{obj.wp_idx});
                    else
                        obj.mission_complete = true;
                        fprintf('[MissionAgent] All waypoints complete.\n');
                    end
                end
            elseif dist < obj.wp_tol
                obj.dwelling = true;
                task_str = sprintf('[WP%d] %s — arrived, initiating task', ...
                    obj.wp_idx, obj.tasks{obj.wp_idx});
                fprintf('[MissionAgent] WP%d reached at t. Task: %s\n', ...
                    obj.wp_idx, obj.tasks{obj.wp_idx});
            else
                task_str = sprintf('[WP%d] %s — en-route, dist=%.1fm', ...
                    obj.wp_idx, obj.tasks{obj.wp_idx}, dist);
            end
        end
    end
end
