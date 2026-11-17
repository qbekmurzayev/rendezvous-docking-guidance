function X = curv_to_cart(Q, a)
% curvilinear state [dr; a*dtheta; a*dphi; rates] -> Cartesian LVLH state

dr = Q(1,:);  th  = Q(2,:)/a;  ph  = Q(3,:)/a;
rd = Q(4,:);  thd = Q(5,:)/a;  phd = Q(6,:)/a;
r  = a + dr;

X = [r.*cos(ph).*cos(th) - a;
     r.*cos(ph).*sin(th);
     r.*sin(ph);
     rd.*cos(ph).*cos(th) - r.*sin(ph).*phd.*cos(th) - r.*cos(ph).*sin(th).*thd;
     rd.*cos(ph).*sin(th) - r.*sin(ph).*phd.*sin(th) + r.*cos(ph).*cos(th).*thd;
     rd.*sin(ph) + r.*cos(ph).*phd];
end
