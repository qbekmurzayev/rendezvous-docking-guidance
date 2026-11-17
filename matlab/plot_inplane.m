function plot_inplane(t, X, tb, ttl, file)
% trajectory in the orbital plane: full mission (km) and last 1.5 km (m)

ib = zeros(size(tb));
for k = 1:numel(tb)
    [~, ib(k)] = min(abs(t - tb(k)));
end

fig = figure('Position', [100 100 1150 450]);
subplot(1,2,1);
plot(X(2,:)/1e3, X(1,:)/1e3, 'b-', 'LineWidth', 1.4); hold on;
plot(X(2,ib)/1e3, X(1,ib)/1e3, 'r^', 'MarkerFaceColor', 'r');
plot(0, 0, 'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'y');
xlabel('Along-track y (km)'); ylabel('Radial x (km)');
title([ttl ', whole mission']); legend('Chaser', 'Burns', 'Target', 'Location', 'best');
grid on;

subplot(1,2,2);
plot(X(2,:), X(1,:), 'b-', 'LineWidth', 1.4); hold on;
plot(X(2,ib), X(1,ib), 'r^', 'MarkerFaceColor', 'r');
plot(0, 0, 'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'y');
axis equal; xlim([-1500 200]); ylim([-900 400]);
xlabel('Along-track y (m)'); ylabel('Radial x (m)');
title('Last 1.5 km (equal axes)'); grid on;

save_fig(fig, file);
end
