function p = params()
% orbit and vehicle data (similar to an ISS cargo vehicle)

p.mu  = 398600.4418e9;   % m^3/s^2
p.Re  = 6378.137e3;      % m
p.g0  = 9.80665;

p.a   = p.Re + 400e3;    % target orbit radius, m
p.inc = deg2rad(51.6);
p.n   = sqrt(p.mu/p.a^3);
p.T   = 2*pi/p.n;

p.mass   = 1000;   % kg
p.Isp    = 220;    % s
p.thrust = 440;    % N

p.fig_dir = fullfile('..', 'results');
end
