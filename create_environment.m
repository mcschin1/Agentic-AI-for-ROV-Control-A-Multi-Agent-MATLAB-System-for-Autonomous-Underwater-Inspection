function env = create_environment()
%CREATE_ENVIRONMENT  Defines the underwater scene: seabed, obstacles, structures

% Obstacles: [cx cy cz radius]  (corals, rock formations, pipeline sections)
env.obstacles = [
    10   5  -8   2.5;    % coral head 1
    20  15  -12  3.0;    % rock formation
    35  -5  -15  2.0;    % debris field
    50  10  -10  4.0;    % subsea structure (large)
    60  20  -18  2.5;    % pipeline section
    75   0  -20  3.5;    % rock outcrop
];

% Seabed profile (simplified flat with slope)
env.seabed_depth = -25;   % [m]
env.water_column = 25;    % [m]
env.bounds = [-5 100 -20 40 -30 0];  % [xmin xmax ymin ymax zmin zmax]

% Currents: [vx vy vz] uniform background current
env.current = [0.1 0.05 0.0];

fprintf('[Environment] Loaded: %d obstacles | depth=%.0fm\n', ...
        size(env.obstacles,1), abs(env.seabed_depth));
end
