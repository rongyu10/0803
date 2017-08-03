function  bd = get_personshape(pos, D)

%-----------
x0 = [pos(1);pos(1);pos(1)];
y0 = [pos(2);pos(2);pos(2)];
deg = pos(3);
c = cos(deg*pi/180);
s = sin(deg*pi/180);
%-----------

%--Default Person Shape---
R = D/2;
theta1  = 0*pi/180;
theta2  = 150*pi/180;
theta3  = 210*pi/180;
thetaPt = [theta1; theta2; theta3];
pointx  = R*cos(thetaPt);
pointy  = R*sin(thetaPt);
%------------------------

x = x0 + pointx*c - pointy*s;
y = y0 + pointx*s + pointy*c;
bd.triangle = [x y];

%------Circle points--------
theta = (0:0.2:2*pi)';
one = ones(length(theta),1);
circlex = R*cos(theta) + one*pos(1);
circley = R*sin(theta) + one*pos(2);
bd.circle = [circlex circley];
%--------------------------
end

