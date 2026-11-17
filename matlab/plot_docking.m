function plot_docking(out, ttl, file)
% terminal approach: lateral offset, range, closing speed, command

s = out.S(1,:);  lat = out.S(2,:);  vc = out.S(4,:);
fig = figure('Position', [100 100 1000 650]);

subplot(2,2,1);
plot(s, lat, 'b-', 'LineWidth', 1.4); hold on;
plot([0 100], [0 100*tand(10)], 'r--');
set(gca, 'XDir', 'reverse'); grid on;
xlabel('Range to port (m)'); ylabel('Lateral offset (m)');
legend('Chaser', '10 deg cone', 'Location', 'northwest');
title(ttl);

subplot(2,2,2);
sref = max((100 + out.vDock/out.k)*exp(-out.k*out.t) - out.vDock/out.k, 0);
plot(out.t, s, 'b-', out.t, sref, 'k--', 'LineWidth', 1.3); grid on;
xlabel('Time (s)'); ylabel('Range (m)'); legend('Actual', 'Glide slope');

subplot(2,2,3);
sg = linspace(0, 100, 50);
plot(s, vc, 'b-', sg, out.vDock + out.k*sg, 'k--', ...
     sg, max(0.2, 0.01*sg), 'r:', 'LineWidth', 1.3);
set(gca, 'XDir', 'reverse'); grid on;
xlabel('Range to port (m)'); ylabel('Closing speed (m/s)');
legend('Actual', 'Glide slope', 'Speed limit', 'Location', 'northwest');

subplot(2,2,4);
plot(out.t, out.U*1000, 'b-', 'LineWidth', 1.2); grid on;
xlabel('Time (s)'); ylabel('Applied thrust (N)');

save_fig(fig, file);
end
