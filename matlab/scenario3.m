function R = scenario3(makePlots)
% scenario 3: terminal approach from 100 m to the docking port

p = params();
fprintf('\n=== SCENARIO 3: terminal approach ===\n');

x0 = [1.5; -100; -1.0; 0.005; 0.295; -0.003];   % start state (m, m/s)
aMax = 0.02;                                     % 20 N on 1000 kg

% nominal run, wn = 0.05 rad/s, zeta = 1
nom = docking_sim(x0, 0.05, 1.0, aMax, [0 0], p.n);
print_run('nominal', nom);
fprintf('  predicted contact time %.1f s\n', nom.tf);

% gain study
wn   = [0.0158, 0.05, sqrt(0.1), 0.05];
zeta = [1.00,   1.00, 0.5/(2*sqrt(0.1)), 0.40];
thr = zeros(1,4);  err = thr;  vc = thr;
fprintf('  Kp        Kd       wn     zeta   t_f     v_c     err    thrust  sat\n');
for k = 1:4
    o = docking_sim(x0, wn(k), zeta(k), aMax, [0 0], p.n);
    thr(k) = o.thrustN;  err(k) = o.maxErr;  vc(k) = o.vAxial;
    fprintf('  %.2e  %.3f  %.3f  %.2f  %.1f  %.4f  %.3f  %6.1f  %.0f%%\n', wn(k)^2, ...
            2*zeta(k)*wn(k), wn(k), zeta(k), o.tEnd, o.vAxial, o.maxErr, o.thrustN, o.satPct);
end

% navigation noise: 2 cm and 5 mm/s (1 sigma)
rng(20240);
nav = docking_sim(x0, 0.05, 1.0, aMax, [0.02 0.005], p.n);
print_run('with navigation noise', nav);

% failure case: 0.4 N thrusters and a cross-track speed at the start
x0bad = [4.0; -100; 0; 0.02; 0.295; 0.055];
bad = docking_sim(x0bad, 0.05, 1.0, 4e-4, [0 0], p.n);
print_run('weak thrusters', bad);

R.dv = nom.dv;  R.time = nom.tEnd;

if makePlots
    plot_docking(nom, 'Scenario 3: nominal approach', 's3_dashboard');
    plot_docking(bad, 'Scenario 3: weak thrusters, abort', 's3_dashboard_abort');

    fig = figure('Position', [100 100 1000 380]);
    lbl = {'0.016', '0.05 (mine)', '0.32', '0.05, \zeta=0.4'};
    subplot(1,3,1); bar(thr); set(gca, 'YScale', 'log', 'XTickLabel', lbl);
    ylabel('Peak commanded thrust (N)'); xlabel('\omega_n (rad/s)'); grid on;
    subplot(1,3,2); bar(err); set(gca, 'XTickLabel', lbl);
    ylabel('Peak tracking error (m)'); xlabel('\omega_n (rad/s)'); grid on;
    subplot(1,3,3); bar(vc); set(gca, 'XTickLabel', lbl); ylim([0 0.12]);
    ylabel('Contact speed (m/s)'); xlabel('\omega_n (rad/s)'); grid on;
    save_fig(fig, 's3_gain_trade');
end
end

function print_run(name, o)
if o.aborted
    fprintf('%s: ABORT at t = %.1f s, lateral offset %.1f m, commanded %.1f N\n', ...
            name, o.tEnd, o.latMiss, o.thrustN);
else
    fprintf('%s: contact at %.1f s (%.1f min), v = %.4f m/s axial, %.4f m/s lateral\n', ...
            name, o.tEnd, o.tEnd/60, o.vAxial, o.vLateral);
    fprintf('  lateral miss %.4f m, max cone angle %.2f deg, dV %.3f m/s, peak thrust %.2f N\n', ...
            o.latMiss, o.maxAngle, o.dv, o.thrustN);
end
end
