function s_new = rov_dynamics(s, thrust, dt)
%ROV_DYNAMICS  Simplified 6-DOF ROV dynamics with hydrodynamic drag
%
%  State vector  s = [x y z roll pitch yaw vx vy vz]
%  Thrust vector    = [Fx Fy Fz Mz]   (body frame forces + yaw moment)

% ── Vehicle parameters ──────────────────────────────────
m      = 12.0;          % mass [kg]
Iz     = 0.8;           % yaw inertia [kg·m²]
Cd_t   = [8 8 12];      % translational drag coefficients
Cd_r   = 2.0;           % rotational drag (yaw)

% ── Unpack state ────────────────────────────────────────
x=s(1); y=s(2); z=s(3);
roll=s(4); pitch=s(5); yaw=s(6);
vx=s(7); vy=s(8); vz=s(9);

% ── Body→World rotation (yaw only, simplified) ──────────
cy = cos(yaw); sy = sin(yaw);
Rot = [cy -sy 0; sy cy 0; 0 0 1];

% ── Forces in world frame ────────────────────────────────
F_body = [thrust(1); thrust(2); thrust(3)];
F_world = Rot * F_body;

% ── Translational acceleration ───────────────────────────
v = [vx; vy; vz];
drag_t = -diag(Cd_t) * (v .* abs(v));
a = (F_world + drag_t) / m;

% ── Yaw dynamics ─────────────────────────────────────────
yaw_rate_prev = 0;  % simplified (no state for yaw rate)
d_yaw = thrust(4)/Iz - Cd_r*yaw_rate_prev;

% ── Integrate (Euler) ────────────────────────────────────
vx_n = vx + a(1)*dt;
vy_n = vy + a(2)*dt;
vz_n = vz + a(3)*dt;

x_n  = x  + vx*dt + 0.5*a(1)*dt^2;
y_n  = y  + vy*dt + 0.5*a(2)*dt^2;
z_n  = z  + vz*dt + 0.5*a(3)*dt^2;

yaw_n  = yaw  + d_yaw*dt;
pitch_n = pitch * 0.98;   % passive damping
roll_n  = roll  * 0.98;

% ── Depth floor (seabed constraint) ─────────────────────
if z_n > 0; z_n = 0; vz_n = 0; end      % surface
if z_n < -50; z_n = -50; vz_n = 0; end  % max depth

s_new = [x_n y_n z_n roll_n pitch_n yaw_n vx_n vy_n vz_n];
end
