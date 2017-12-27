%{
theta = 0:0.01:2*pi;

D = 5000;
R = D/2;

x = R*cos(theta);
y = R*sin(theta);

%fill(x,y,'r')
col1 = [0 1 1];
col2 = [0 0 1];

hold on
plot(x,y,'-', 'Color', col2, 'LineWidth', 2)


theta1  = 0*pi/180;
theta2  = 155*pi/180;
theta3  = 205*pi/180;
thetaPt = [theta1, theta2, theta3];
px = R*cos(thetaPt);
py = R*sin(thetaPt);
fill(px,py,col2)
%}

D = 1500;
R = D/2;

pos(1) = 500;
pos(2) = 500;
%------Circle points--------
theta = (0:0.01:2*pi)';
one = ones(length(theta),1);
circlex = R*cos(theta) + one*pos(1);
circley = R*sin(theta) + one*pos(2);
bd_circle = [circlex circley];
plot(bd_circle(:,1),bd_circle(:,2),'-', 'Color', [0 0 1], 'LineWidth', 2)
%--------------------------

