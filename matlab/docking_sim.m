function out = docking_sim(x0, wn, zeta, aMax, sigma, n)
% terminal approach along the V-bar: glide slope + PD control with feedforward
% x0     initial state [r; v] (m, m/s), port at the origin, approach along -y
% wn, zeta  closed-loop natural frequency and damping
% aMax   acceleration limit (m/s^2), sigma = [pos vel] navigation noise (1 sigma)

s0 = 100;  vStart = 0.30;  vDock = 0.10;   % glide slope
ax = [0; -1; 0];                           % approach axis
dt = 0.1;                                  % 10 Hz controller
Kp = wn^2;
Kd = 2*zeta*wn;
k  = (vStart - vDock)/s0;
tf = log(1 + k*s0/vDock)/k;                % predicted contact time
G  = [3*n^2 0 0; 0 0 0; 0 0 -n^2];
H  = [0 2*n 0; -2*n 0 0; 0 0 0];
A  = [zeros(3) eye(3); G H];

N = floor((1.5*tf + 120)/dt) + 1;
t = (0:N-1)*dt;
X = zeros(6,N);  X(:,1) = x0(:);
U = zeros(1,N);  Uraw = zeros(1,N);  E = zeros(1,N);
S = zeros(4,N);                            % s, lateral, angle, closing speed
sat = false(1,N);
dv = 0;  aborted = false;  kEnd = N;

for i = 1:N-1
    xm = X(:,i);
    if any(sigma > 0)
        xm = xm + [sigma(1)*randn(3,1); sigma(2)*randn(3,1)];
    end
    [u, ur, e] = pd_law(t(i), xm);
    [S(:,i), ok] = monitor(X(:,i));
    U(i) = norm(u);  Uraw(i) = norm(ur);  E(i) = e;
    sat(i) = norm(ur) > aMax;
    if ~ok
        aborted = true;  kEnd = i;
        break
    end
    dv = dv + norm(u)*dt;

    f  = @(x) A*x + [0; 0; 0; u];          % RK4, u held over the step
    x  = X(:,i);
    k1 = f(x);  k2 = f(x + dt/2*k1);  k3 = f(x + dt/2*k2);  k4 = f(x + dt*k3);
    X(:,i+1) = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);

    if dot(X(1:3,i+1), ax) <= 0.02         % reached the port plane
        kEnd = i+1;
        break
    end
end

[~, ur, e] = pd_law(t(kEnd), X(:,kEnd));
S(:,kEnd) = monitor(X(:,kEnd));
Uraw(kEnd) = norm(ur);  E(kEnd) = e;

idx = 1:kEnd;
v = X(4:6,kEnd);
out.t = t(idx);  out.X = X(:,idx);  out.U = U(idx);
out.S = S(:,idx);  out.tf = tf;  out.k = k;  out.vDock = vDock;
out.aborted   = aborted;
out.tEnd      = t(kEnd);
out.vAxial    = -dot(v, ax);
out.vLateral  = norm(v - dot(v, ax)*ax);
out.latMiss   = S(2,kEnd);
out.dv        = dv;
out.thrustN   = max(Uraw(idx)) * 1000;     % commanded, before saturation
out.satPct    = 100*mean(sat(idx));
out.maxErr    = max(E(idx));
far = S(1,idx) > 1;
out.maxAngle  = max([0, S(3,idx(far))]);

    function [u, ur, e] = pd_law(tt, x)
        s  = max((s0 + vDock/k)*exp(-k*tt) - vDock/k, 0);   % glide slope s(t)
        vr = vDock + k*s;
        rRef = s*ax;  vRef = -vr*ax;  aRef = k*vr*ax;
        er = x(1:3) - rRef;
        ur = aRef - G*x(1:3) - H*x(4:6) - Kp*er - Kd*(x(4:6) - vRef);
        u  = ur;
        if norm(u) < 2e-5, u = zeros(3,1); end              % deadband
        if norm(u) > aMax, u = aMax*u/norm(u); end           % saturation
        e = norm(er);
    end

    function [st, ok] = monitor(x)
        s   = dot(x(1:3), ax);
        lat = norm(x(1:3) - s*ax);
        ang = 0;
        if s > 1e-6, ang = atan2d(lat, s); end
        vLim = max(0.20, 0.010*s);
        st = [s; lat; ang; -dot(x(4:6), ax)];
        ok = (ang <= 10 || s < 1) && norm(x(4:6)) <= vLim;
    end
end
