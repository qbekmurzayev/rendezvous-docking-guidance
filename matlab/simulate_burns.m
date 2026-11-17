function [t, X] = simulate_burns(x0, burns, dt, n)
% impulsive burns with HCW coasts in between
% burns: one row per burn [t, dvx, dvy, dvz]; the run ends at the last burn

tb    = burns(:,1)';
nodes = unique([0, tb]);
x = x0(:);
t = 0;
X = x;

for s = 1:numel(nodes)
    for b = find(abs(tb - nodes(s)) < 1e-9)
        x(4:6) = x(4:6) + burns(b,2:4)';
    end
    if s == 1 || s == numel(nodes)
        X(:,end) = x;
    end
    if s == numel(nodes), break; end

    ts = nodes(s):dt:nodes(s+1);
    if ts(end) < nodes(s+1), ts(end+1) = nodes(s+1); end
    Xs = hcw_propagate(x, ts - nodes(s), n);
    t  = [t, ts(2:end)];
    X  = [X, Xs(:,2:end)];
    x  = Xs(:,end);
end
end
