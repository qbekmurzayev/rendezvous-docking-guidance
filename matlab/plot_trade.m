function plot_trade(orbits, dvT, dv1, dv2, nRev, dvSel, file)
% two-impulse cost versus transfer time

fig = figure('Position', [100 100 800 450]);
plot(orbits, dvT, 'b-', 'LineWidth', 1.5); hold on;
plot(orbits, dv1, 'g--', orbits, dv2, 'm:', 'LineWidth', 1.1);
plot(nRev, dvSel, 'ro', 'MarkerFaceColor', 'r');
yl = [0, 8*min(dvT)];
for k = 1:floor(max(orbits))
    plot([k k], yl, '-', 'Color', [1 0.6 0.6]);
end
ylim(yl);
xlabel('Transfer time (orbits)'); ylabel('\DeltaV (m/s)');
title('Scenario 1: \DeltaV versus transfer time (50 km to 1 km)');
legend('Total \DeltaV', '|\Deltav_1|', '|\Deltav_2|', 'Selected (3 orbits)', ...
       'Location', 'northeast');
grid on;
save_fig(fig, file);
end
