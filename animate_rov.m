function animate_rov(t, state, mission, env, thrust_log, sonar_log, fault_log)
%ANIMATE_ROV  Rich 3-D animated visualisation of the agentic ROV mission
%
%  Features:
%   - 3-D trajectory with depth-coded colouring
%   - ROV body drawn as patch at current position
%   - Thruster force arrows
%   - Obstacle spheres
%   - Waypoint markers
%   - Live telemetry panel (4 sub-plots)
%   - Agent status overlays

N = length(t);
dt = t(2) - t(1);
skip = max(1, round(0.04/dt));   % animate at ~25 fps

%% ── Figure layout ────────────────────────────────────────
fig = figure('Name','Agentic ROV Control — Live Mission', ...
             'Color',[0.06 0.08 0.12], ...
             'Position',[50 50 1400 820], ...
             'Renderer','opengl');

% Main 3-D view
ax3d = subplot('Position',[0.02 0.30 0.56 0.65]);
hold(ax3d,'on'); grid(ax3d,'on'); box(ax3d,'on');
set(ax3d,'Color',[0.04 0.06 0.10],'GridColor',[0.3 0.4 0.5],...
         'XColor',[0.7 0.8 0.9],'YColor',[0.7 0.8 0.9],...
         'ZColor',[0.7 0.8 0.9],'LineWidth',0.5);
xlabel(ax3d,'East (m)','Color',[0.8 0.9 1],'FontSize',10);
ylabel(ax3d,'North (m)','Color',[0.8 0.9 1],'FontSize',10);
zlabel(ax3d,'Depth (m)','Color',[0.8 0.9 1],'FontSize',10);
title(ax3d,'Agentic ROV — Underwater Mission','Color',[0.9 1.0 1.0],...
      'FontSize',13,'FontWeight','bold');
view(ax3d, 35, 22);

% Telemetry subplots
ax_thrust = subplot('Position',[0.62 0.72 0.36 0.22]);
ax_sonar  = subplot('Position',[0.62 0.43 0.36 0.22]);
ax_depth  = subplot('Position',[0.62 0.14 0.36 0.22]);
ax_fault  = subplot('Position',[0.02 0.02 0.56 0.22]);

%% ── Draw static environment ──────────────────────────────
wps = mission.waypoints;

% Seabed plane
[Xsb, Ysb] = meshgrid(linspace(-5,90,15), linspace(-25,35,12));
Zsb = env.seabed_depth * ones(size(Xsb));
surf(ax3d, Xsb, Ysb, Zsb, 'FaceColor',[0.35 0.28 0.18], ...
     'EdgeColor',[0.5 0.4 0.3],'FaceAlpha',0.6,'EdgeAlpha',0.3);

% Obstacle spheres
[Ssx,Ssy,Ssz] = sphere(20);
obs_colors = {[0.9 0.4 0.1],[0.6 0.6 0.7],[0.5 0.7 0.4],...
              [0.3 0.5 0.8],[0.8 0.6 0.2],[0.7 0.4 0.6]};
for i = 1:size(env.obstacles,1)
    cx=env.obstacles(i,1); cy=env.obstacles(i,2);
    cz=env.obstacles(i,3); r=env.obstacles(i,4);
    col = obs_colors{mod(i-1,length(obs_colors))+1};
    surf(ax3d, cx+r*Ssx, cy+r*Ssy, cz+r*Ssz, ...
         'FaceColor',col,'EdgeColor','none','FaceAlpha',0.55);
end

% Waypoints
for i = 1:size(wps,1)
    plot3(ax3d, wps(i,1), wps(i,2), wps(i,3), ...
          's','Color',[0.2 1.0 0.4],'MarkerSize',10,...
          'MarkerFaceColor',[0.2 1.0 0.4],'LineWidth',1.5);
    text(ax3d, wps(i,1)+1, wps(i,2)+1, wps(i,3)+0.5, ...
         sprintf('WP%d',i),'Color',[0.6 1.0 0.7],'FontSize',8,'FontWeight','bold');
end
% Connect waypoints with dashed line
plot3(ax3d, wps(:,1), wps(:,2), wps(:,3), '--', ...
      'Color',[0.3 0.8 0.4],'LineWidth',0.8,'LineStyle',':');

% Colour-mapped trajectory trail (drawn incrementally)
h_trail = plot3(ax3d, NaN,NaN,NaN, '-', 'Color',[0.2 0.7 1.0],'LineWidth',2);

% Thrust arrows
h_arr_x = quiver3(ax3d,0,0,0,0,0,0,'r','LineWidth',1.5,'MaxHeadSize',0.8);
h_arr_y = quiver3(ax3d,0,0,0,0,0,0,'g','LineWidth',1.5,'MaxHeadSize',0.8);
h_arr_z = quiver3(ax3d,0,0,0,0,0,0,'b','LineWidth',1.5,'MaxHeadSize',0.8);

% ROV body (simplified box)
h_rov = draw_rov_body(ax3d, state(1,1:3), state(1,6));

% Set axis limits
axis(ax3d,[-5 90 -25 40 -30 2]);
lighting(ax3d,'gouraud'); camlight(ax3d,'headlight');

%% ── Telemetry plot setup ─────────────────────────────────
style_telem(ax_thrust,'Thrust [N]','Fx','Fy','Fz',[0.6 0.08 0.12]);
style_telem(ax_sonar, 'Sonar [m]','Obs Dist','','',[0.08 0.12 0.06]);
style_telem(ax_depth, 'Depth [m]','Depth','','',[0.08 0.10 0.16]);
style_telem(ax_fault, 'System status','Fault Flag','','',[0.10 0.06 0.06]);

h_Fx  = line(ax_thrust,'XData',[],'YData',[],'Color',[1.0 0.4 0.4],'LineWidth',1.2);
h_Fy  = line(ax_thrust,'XData',[],'YData',[],'Color',[0.4 1.0 0.4],'LineWidth',1.2);
h_Fz  = line(ax_thrust,'XData',[],'YData',[],'Color',[0.4 0.6 1.0],'LineWidth',1.2);
h_son = line(ax_sonar,'XData',[],'YData',[],'Color',[0.9 0.8 0.2],'LineWidth',1.5);
h_dep = line(ax_depth,'XData',[],'YData',[],'Color',[0.3 0.8 1.0],'LineWidth',1.5);
h_flt = line(ax_fault,'XData',[],'YData',[],'Color',[1.0 0.5 0.2],'LineWidth',1.5);

% Legend
legend(ax_thrust,'Fx','Fy','Fz','TextColor',[0.9 0.9 0.9],'FontSize',8,...
       'Location','northwest','Color',[0.1 0.12 0.15],'EdgeColor','none');

% Status text overlays
h_status = annotation(fig,'textbox',[0.62 0.00 0.36 0.12],...
    'String','Initialising...','Color',[0.8 1.0 0.8],...
    'BackgroundColor',[0.05 0.08 0.10],'EdgeColor',[0.2 0.4 0.3],...
    'FontSize',8,'FontName','Courier New','FitBoxToText','off');

h_title_status = annotation(fig,'textbox',[0.02 0.92 0.96 0.06],...
    'String','AGENTIC ROV CONTROL SYSTEM — MISSION IN PROGRESS',...
    'Color',[0.9 1.0 1.0],'BackgroundColor',[0.08 0.12 0.18],...
    'EdgeColor',[0.3 0.5 0.7],'FontSize',11,'FontWeight','bold',...
    'HorizontalAlignment','center','FitBoxToText','off');

drawnow;

%% ── Animation loop ────────────────────────────────────────
t_buf  = [];
Fx_buf = []; Fy_buf = []; Fz_buf = [];
son_buf= []; dep_buf= []; flt_buf= [];
trail_x=[]; trail_y=[]; trail_z=[];

for k = 1:skip:N-1
    pos = state(k,1:3);
    yaw = state(k,6);

    % Trajectory trail (depth-colour coded)
    trail_x(end+1)=pos(1); trail_y(end+1)=pos(2); trail_z(end+1)=pos(3); %#ok
    alpha_val = min(1, max(0.3, 1 + pos(3)/25));
    set(h_trail,'XData',trail_x,'YData',trail_y,'ZData',trail_z,...
                'Color',[0.2 alpha_val*0.9 1.0]);

    % Update ROV body
    delete(h_rov);
    h_rov = draw_rov_body(ax3d, pos, yaw);

    % Thrust arrows
    sc = 0.3;
    set(h_arr_x,'XData',pos(1),'YData',pos(2),'ZData',pos(3),...
        'UData',thrust_log(k,1)*sc,'VData',0,'WData',0);
    set(h_arr_y,'XData',pos(1),'YData',pos(2),'ZData',pos(3),...
        'UData',0,'VData',thrust_log(k,2)*sc,'WData',0);
    set(h_arr_z,'XData',pos(1),'YData',pos(2),'ZData',pos(3),...
        'UData',0,'VData',0,'WData',thrust_log(k,3)*sc);

    % Telemetry buffers
    t_buf(end+1)  = t(k);   %#ok
    Fx_buf(end+1) = thrust_log(k,1); %#ok
    Fy_buf(end+1) = thrust_log(k,2); %#ok
    Fz_buf(end+1) = thrust_log(k,3); %#ok
    son_buf(end+1)= sonar_log(k);    %#ok
    dep_buf(end+1)= pos(3);          %#ok
    flt_buf(end+1)= fault_log(k);    %#ok

    set(h_Fx,'XData',t_buf,'YData',Fx_buf);
    set(h_Fy,'XData',t_buf,'YData',Fy_buf);
    set(h_Fz,'XData',t_buf,'YData',Fz_buf);
    set(h_son,'XData',t_buf,'YData',son_buf);
    set(h_dep,'XData',t_buf,'YData',dep_buf);
    set(h_flt,'XData',t_buf,'YData',flt_buf);

    % Axis limits roll with time
    tmax = max(t_buf); tmin = max(0, tmax-20);
    set(ax_thrust,'XLim',[tmin tmax+1]);
    set(ax_sonar, 'XLim',[tmin tmax+1]);
    set(ax_depth, 'XLim',[tmin tmax+1]);
    set(ax_fault, 'XLim',[tmin tmax+1]);

    % Status panel
    fault_str = 'OK'; if fault_log(k)>0; fault_str='⚠ FAULT ACTIVE'; end
    set(h_status,'String', sprintf([ ...
        'Time:  %.1f s\n' ...
        'Pos:   (%.1f, %.1f, %.1f) m\n' ...
        'Sonar: %.1f m\n' ...
        'Fault: %s\n' ...
        'Thrust:[%.1f  %.1f  %.1f] N'], ...
        t(k), pos(1),pos(2),pos(3), sonar_log(k), fault_str, ...
        thrust_log(k,1),thrust_log(k,2),thrust_log(k,3)));

    drawnow limitrate;
end

%% ── Final frame ──────────────────────────────────────────
set(h_title_status,'String', ...
    'AGENTIC ROV CONTROL SYSTEM — MISSION COMPLETE ✓', ...
    'Color',[0.4 1.0 0.6]);
drawnow;
fprintf('[Animate] Rendered %d frames.\n', ceil(N/skip));
end


%% ── Helper: draw ROV body ────────────────────────────────
function h = draw_rov_body(ax, pos, yaw)
    Lx=1.8; Ly=1.0; Lz=0.6;
    verts = Lx/2*[-1 -1 -1 -1 1 1 1 1]' * [1 0 0] + ...
            Ly/2*[-1 -1  1  1 -1 -1 1 1]' * [0 1 0] + ...
            Lz/2*[-1  1 -1  1 -1  1 -1 1]' * [0 0 1];
    cy=cos(yaw); sy=sin(yaw);
    R=[cy -sy 0; sy cy 0; 0 0 1];
    verts = (R*verts')';
    verts = verts + pos;
    faces=[1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];
    h = patch(ax,'Vertices',verts,'Faces',faces,...
              'FaceColor',[0.2 0.5 0.8],'EdgeColor',[0.6 0.8 1.0],...
              'FaceAlpha',0.85,'LineWidth',0.8);
end


%% ── Helper: style a telemetry subplot ────────────────────
function style_telem(ax, ylbl, l1, l2, l3, bg)
    hold(ax,'on'); grid(ax,'on');
    set(ax,'Color',bg,'GridColor',[0.25 0.35 0.30],...
           'XColor',[0.7 0.8 0.7],'YColor',[0.7 0.8 0.7],...
           'LineWidth',0.5,'FontSize',8);
    ylabel(ax,ylbl,'Color',[0.8 0.9 0.8],'FontSize',8);
    xlabel(ax,'Time (s)','Color',[0.6 0.7 0.6],'FontSize',7);
end
