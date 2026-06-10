function mission = create_mission(env)
%CREATE_MISSION  Defines inspection waypoints and task descriptions

% Waypoints [x y z]  — survey pattern at varying depths
mission.waypoints = [
     0    0   -3;    % WP1: Launch & descent
    15    5  -10;    % WP2: Pipeline inspection start
    30    8  -14;    % WP3: Coral survey
    45   10  -12;    % WP4: Structure inspection
    60   15  -18;    % WP5: Seabed sampling
    75    5  -20;    % WP6: Deep survey
    80    0  -10;    % WP7: Ascent waypoint
    80    0   -2;    % WP8: Surface rendezvous
];

mission.tasks = {
    'Launch and descend'
    'Pipeline visual inspection'
    'Coral reef survey'
    'Subsea structure NDT'
    'Seabed sediment sampling'
    'Deep water survey'
    'Controlled ascent'
    'Surface recovery'
};

mission.n_wp = size(mission.waypoints, 1);

fprintf('[Mission] Loaded %d-waypoint inspection survey.\n', mission.n_wp);
end
