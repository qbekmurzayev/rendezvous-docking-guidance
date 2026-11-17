function R = scenario1(makePlots)
% scenario 1: chaser on the same orbit, 50 km behind the target

p = params();
n = p.n;  T = p.T;
fprintf('\n=== SCENARIO 1: same orbit, 50 km behind ===\n');

q0   = [0; -50e3; 0; 0; 0; 0];   % curvilinear state (50 km of arc behind)
wp1  = [0; -1e3; 0];             % hold point
wp2  = [0; -100; 0];             % start of terminal approach
nRev = 3;
tHold = 15*60;  tCheck = 5*60;

% leg 1: phasing, closed form (dV*t = 2/3*dy)
dy  = wp1(2) - q0(2);
dv  = dy / (3*T*nRev);                    % one burn
da  = -2*dv/n;                            % lower orbit
tf1 = nRev * T*(1 + 1.5*da/p.a);
fprintf('Phasing: %d revs, orbit %.0f m lower, %.2f h\n', nRev, abs(da), tf1/3600);
fprintf('  closed form   dV = %.4f m/s (2 x %.4f)\n', 2*dv, dv);

% same leg with the general two-impulse solver
[b1, b2, c] = two_impulse(q0, wp1, tf1, n);
dvCW = norm(b1) + norm(b2);
fprintf('  two-impulse   dV = %.4f m/s, cond = %.0f, difference %.3f %%\n', ...
        dvCW, c, 100*abs(dvCW - 2*dv)/(2*dv));

% leg 2: radial hop 1 km -> 100 m (half an orbit)
tf2 = T/2;
[b3, b4] = two_impulse([wp1; 0; 0; 0], wp2, tf2, n);
dvx = n*abs(wp2(2) - wp1(2))/4;
fprintf('Radial hop: 2 x %.4f m/s, goes %.0f m below the V-bar\n', dvx, dvx/n);

% burn plan [t, dv]
t3 = tf1 + tHold;  t5 = t3 + tf2 + tCheck;
burns = [0,        b1';
         tf1,      b2';
         t3,       b3';
         t3 + tf2, b4';
         t5,       0, 0.30, 0];
[t, Q] = simulate_burns(q0, burns, 1, n);
X = curv_to_cart(Q, p.a);

dvTot = sum(sqrt(sum(burns(:,2:4).^2, 2)));
mProp = p.mass*(1 - exp(-dvTot/(p.Isp*p.g0)));
vMax  = max(sqrt(sum(X(4:6,:).^2)));
tBurn = max(sqrt(sum(burns(:,2:4).^2, 2))) * p.mass/p.thrust;
fprintf('Total: %.3f m/s, %.3f kg, %.2f h\n', dvTot, mProp, t5/3600);
fprintf('Max relative speed %.3f m/s, longest burn %.2f s at %.3f g\n', ...
        vMax, tBurn, p.thrust/p.mass/p.g0);

% phasing trade
fprintf('Phasing trade (dV*t = %.0f m):\n', dvCW*tf1);
for k = [2 3 4 6 8]
    fprintf('  %d revs: %.3f m/s, %.2f h\n', k, 2*dy/(3*T*k), k*T*(1 - dy/(T*k*n*p.a))/3600);
end
fprintf('  0.44 m/s would need %.1f h\n', 2*dy/(3*0.44)/3600);

% sweep of the transfer time
tfv = linspace(0.35*T, 8*T, 1600);
dvT = nan(size(tfv));  d1 = dvT;  d2 = dvT;
for k = 1:numel(tfv)
    [a1, a2, ck] = two_impulse(q0, wp1, tfv(k), n);
    if ck < 1e6
        d1(k) = norm(a1);  d2(k) = norm(a2);  dvT(k) = d1(k) + d2(k);
    end
end

% validation against the nonlinear model during the phasing coast
q1  = q0;  q1(4:6) = b1;
tv  = linspace(0, tf1, 1200);
x1  = curv_to_cart(q1, p.a);
Xnl = nonlinear_relative(x1, tv, p);
Xcu = curv_to_cart(hcw_propagate(q1, tv, n), p.a);
Xca = hcw_propagate(x1, tv, n);
rng_nl = sqrt(sum(Xnl(1:3,:).^2));
ecu = sqrt(sum((Xcu(1:3,:) - Xnl(1:3,:)).^2));
eca = sqrt(sum((Xca(1:3,:) - Xnl(1:3,:)).^2));
fprintf('Validation: curvilinear max error %.1f m (%.2f %%), Cartesian %.1f km, ratio %.0f\n', ...
        max(ecu), 100*max(ecu./rng_nl), max(eca)/1e3, max(eca)/max(ecu));
fprintf('  semi-major axis error of the Cartesian start: %.0f m\n', ...
        4*p.a*(1 - cos(50e3/p.a)));

R.dv = dvTot;  R.mProp = mProp;  R.time = t5;

if makePlots
    plot_inplane(t, Q, burns(:,1), 'Scenario 1', 's1_inplane');
    plot_trade(tfv/T, dvT, d1, d2, tf1/T, dvCW, 's1_dv_trade');
    plot_validation(tv, Xnl, Xcu, Xca, T, 's1_validation');
end
end
