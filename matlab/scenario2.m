function R = scenario2(makePlots)
% scenario 2: chaser 2.5 km below and 100 km behind, small inclination difference

p = params();
n = p.n;  T = p.T;
fprintf('\n=== SCENARIO 2: lower orbit, 100 km behind ===\n');

dAlt = -2.5e3;  y0 = -100e3;  zAmp = 500;
wp1  = [0; -1e3; 0];  wp2 = [0; -100; 0];
tHold = 15*60;  tCheck = 5*60;

ydrift = -1.5*n*dAlt;                     % free co-elliptic drift
x0 = [dAlt; y0; zAmp; 0; ydrift; 0];      % curvilinear state
fprintf('Drift speed %.3f m/s, relative inclination %.4f deg\n', ...
        ydrift, rad2deg(zAmp/p.a));

% burn 1: remove cross-track motion at the node (z = 0 after T/4)
tNode = T/4;
xN  = hcw_propagate(x0, tNode, n);
dvz = [0; 0; -xN(6)];
fprintf('Plane correction at t = %.0f s: %.4f m/s\n', tNode, norm(dvz));

% where to stop the drift so that a tangential Hohmann transfer ends at wp1
tf3 = T/2;
dvH = 0.25*n*abs(dAlt);                   % per burn
xDep  = hcw_propagate([wp1; 0; -dvH; 0], -tf3, n);
yStop = xDep(2);

% free drift until y = yStop
xA = xN;  xA(4:6) = xA(4:6) + dvz;
ts = 0:10:8*T;
Xs = hcw_propagate(xA, ts, n);
i  = find(Xs(2,:) >= yStop, 1);
tStop = ts(i-1) + (yStop - Xs(2,i-1))/(Xs(2,i) - Xs(2,i-1))*(ts(i) - ts(i-1));
xStop = hcw_propagate(xA, tStop, n);
fprintf('Free drift %.2f h, stop at %.2f km\n', tStop/3600, abs(yStop)/1e3);

% Hohmann transfer and radial hop
[b2, b3] = two_impulse(xStop, wp1, tf3, n);
fprintf('Hohmann: %.4f m/s (formula %.4f m/s)\n', norm(b2) + norm(b3), 2*dvH);
[b4, b5] = two_impulse([wp1; 0; 0; 0], wp2, T/2, n);

tB = tNode + tStop;  tD = tB + tf3 + tHold;  tF = tD + T/2 + tCheck;
burns = [tNode,     dvz';
         tB,        b2';
         tB + tf3,  b3';
         tD,        b4';
         tD + T/2,  b5';
         tF,        0, 0.30, 0];
[t, Q] = simulate_burns(x0, burns, 2, n);

dvTot = sum(sqrt(sum(burns(:,2:4).^2, 2)));
mProp = p.mass*(1 - exp(-dvTot/(p.Isp*p.g0)));
fprintf('Total: %.3f m/s, %.3f kg, %.2f h\n', dvTot, mProp, tF/3600);

% linear model error during the drift and the correction it needs
tv  = linspace(0, tStop, 1500);
xc  = curv_to_cart(xA, p.a);
Xnl = nonlinear_relative(xc, tv, p);
Xcu = curv_to_cart(hcw_propagate(xA, tv, n), p.a);
Xca = hcw_propagate(xc, tv, n);
ecu = max(sqrt(sum((Xcu(1:3,:) - Xnl(1:3,:)).^2)));
eca = max(sqrt(sum((Xca(1:3,:) - Xnl(1:3,:)).^2)));
dvMid = norm(Xcu(4:6,end) - Xnl(4:6,end));
fprintf('Drift error: curvilinear %.1f m, Cartesian %.0f km, correction %.4f m/s\n', ...
        ecu, eca/1e3, dvMid);

R.dv = dvTot;  R.time = tF;

if makePlots
    plot_inplane(t, Q, burns(:,1), 'Scenario 2', 's2_inplane');
end
end
