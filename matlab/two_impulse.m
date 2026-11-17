function [dv1, dv2, c] = two_impulse(x0, rf, tf, n)
% two-impulse HCW transfer: from state x0 to position rf in time tf,
% arriving with zero relative velocity. c = condition number (in-plane)

[Prr, Prv, Pvr, Pvv] = hcw_stm(tf, n);
r0 = x0(1:3);
v0 = x0(4:6);
b  = rf(:) - Prr*r0;

c = cond(Prv(1:2,1:2));
if c > 1e10            % tf is a whole number of orbits
    dv1 = nan(3,1);
    dv2 = nan(3,1);
    return
end

v0p = zeros(3,1);
v0p(1:2) = Prv(1:2,1:2) \ b(1:2);   % in-plane
if abs(Prv(3,3)) > 1e-9
    v0p(3) = b(3) / Prv(3,3);        % cross-track
else
    v0p(3) = v0(3);
end

dv1 = v0p - v0;
dv2 = -(Pvr*r0 + Pvv*v0p);
end
