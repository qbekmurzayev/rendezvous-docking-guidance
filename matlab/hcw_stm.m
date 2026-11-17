function [Prr, Prv, Pvr, Pvv] = hcw_stm(t, n)
% HCW state transition matrix, split in 3x3 blocks
% x radial, y along track, z cross track

c = cos(n*t);
s = sin(n*t);

Prr = [4-3*c,        0, 0;
       6*(s-n*t),    1, 0;
       0,            0, c];

Prv = [ s/n,          2*(1-c)/n,       0;
       -2*(1-c)/n,   (4*s-3*n*t)/n,    0;
        0,            0,               s/n];

Pvr = [ 3*n*s,        0, 0;
       -6*n*(1-c),    0, 0;
        0,            0, -n*s];

Pvv = [ c,     2*s,      0;
       -2*s,   4*c-3,    0;
        0,     0,        c];
end
