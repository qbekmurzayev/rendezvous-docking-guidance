% Project B: rendezvous and docking guidance in LEO
% runs the three scenarios; figures are saved to ../results

clear; close all; clc;
makePlots = true;

R1 = scenario1(makePlots);
R2 = scenario2(makePlots);
R3 = scenario3(makePlots);

p = params();
dv = R2.dv + R3.dv;
fprintf('\n=== FULL MISSION (scenario 2 + scenario 3) ===\n');
fprintf('Delta-V %.3f m/s, time %.2f h, propellant %.3f kg\n', ...
        dv, (R2.time + R3.time)/3600, p.mass*(1 - exp(-dv/(p.Isp*p.g0))));
