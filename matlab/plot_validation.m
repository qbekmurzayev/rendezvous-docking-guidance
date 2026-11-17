function plot_validation(t, Xnl, Xcu, Xca, T, file)
% linear HCW (curvilinear and Cartesian) against the nonlinear model

ecu = sqrt(sum((Xcu(1:3,:) - Xnl(1:3,:)).^2));
eca = sqrt(sum((Xca(1:3,:) - Xnl(1:3,:)).^2));

fig = figure('Position', [100 100 900 650]);
subplot(2,1,1);
plot(Xnl(2,:)/1e3, Xnl(1,:)/1e3, 'k-', 'LineWidth', 2); hold on;
plot(Xcu(2,:)/1e3, Xcu(1,:)/1e3, 'g--', 'LineWidth', 1.4);
plot(Xca(2,:)/1e3, Xca(1,:)/1e3, 'r:', 'LineWidth', 1.6);
xlabel('Along-track (km)'); ylabel('Radial (km)');
legend('Nonlinear (truth)', 'Curvilinear HCW', 'Cartesian HCW', 'Location', 'best');
title('Scenario 1: phasing coast'); grid on;

subplot(2,1,2);
semilogy(t/T, max(ecu, 1e-2), 'g-', t/T, max(eca, 1e-2), 'r-', 'LineWidth', 1.4);
xlabel('Time (orbits)'); ylabel('Position error (m)');
legend(sprintf('Curvilinear (max %.1f m)', max(ecu)), ...
       sprintf('Cartesian (max %.0f m)', max(eca)), 'Location', 'southeast');
grid on;
save_fig(fig, file);
end
