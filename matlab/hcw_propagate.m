function X = hcw_propagate(x0, t, n)
% propagates the state x0 = [r; v] to all times in t (no control)

X = zeros(6, numel(t));
for k = 1:numel(t)
    [Prr, Prv, Pvr, Pvv] = hcw_stm(t(k), n);
    X(:,k) = [Prr Prv; Pvr Pvv] * x0(:);
end
end
