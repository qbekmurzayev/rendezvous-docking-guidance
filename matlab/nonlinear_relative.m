function X = nonlinear_relative(x0, t, p)
% "truth" model: target and chaser follow two-body gravity in ECI,
% the result is converted back to the LVLH frame.
% x0 is the Cartesian LVLH state of the chaser at t = 0.

rt = p.a * [1; 0; 0];                            % target at t = 0
vt = p.a*p.n * [0; cos(p.inc); sin(p.inc)];
R  = lvlh(rt, vt);
w  = [0; 0; p.n];
rc = rt + R*x0(1:3);
vc = vt + R*(x0(4:6) + cross(w, x0(1:3)));

opts = odeset('RelTol', 1e-11, 'AbsTol', 1e-8);
[~, Y] = ode45(@(tt, y) two_body(y, p.mu), t, [rc; vc; rt; vt], opts);

X = zeros(6, numel(t));
for k = 1:numel(t)
    rc = Y(k,1:3)';  vc = Y(k,4:6)';
    rt = Y(k,7:9)';  vt = Y(k,10:12)';
    R  = lvlh(rt, vt);
    w  = [0; 0; norm(cross(rt, vt))/norm(rt)^2];
    r  = R' * (rc - rt);
    X(:,k) = [r; R'*(vc - vt) - cross(w, r)];
end
end

function dy = two_body(y, mu)
rc = y(1:3);  rt = y(7:9);
dy = [y(4:6); -mu*rc/norm(rc)^3; y(10:12); -mu*rt/norm(rt)^3];
end

function R = lvlh(r, v)
x = r/norm(r);
z = cross(r, v);  z = z/norm(z);
R = [x, cross(z, x), z];
end
