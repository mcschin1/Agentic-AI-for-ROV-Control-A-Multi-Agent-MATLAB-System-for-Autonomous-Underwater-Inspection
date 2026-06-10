%% ── SonarAgent ─────────────────────────────────────────────
classdef SonarAgent < handle
%SONARAGENT  Simulates multibeam sonar; returns range to nearest obstacle

    properties
        obstacles
        range_max = 20;     % sonar max range [m]
        noise_std = 0.2;    % measurement noise [m]
        n_beams   = 16;     % number of sonar beams
    end

    methods
        function obj = SonarAgent(obstacles)
            obj.obstacles = obstacles;
            fprintf('[SonarAgent] Initialised with %d obstacles, %d beams.\n', ...
                    size(obstacles,1), obj.n_beams);
        end

        function [min_dist, obs_dir] = sense(obj, pos, obstacles)
            min_dist = obj.range_max;
            obs_dir  = [0 0 0];
            for i = 1:size(obstacles,1)
                obs_center = obstacles(i,1:3);
                obs_radius = obstacles(i,4);
                vec  = obs_center - pos;
                dist = norm(vec) - obs_radius;
                if dist < min_dist
                    min_dist = max(dist, 0.1);
                    obs_dir  = vec / (norm(vec)+1e-6);
                end
            end
            % Add Gaussian noise
            min_dist = min_dist + obj.noise_std * randn();
            min_dist = max(0.1, min_dist);
        end
    end
end
