%-- Area Setting----
ANGLE  = 16*(pi/180); % 16 degree
RADIUS = 30;     % 30 m
%--------------------
theta  = linspace(-0.5*ANGLE, 0.5*ANGLE, 20)';
arc_x = RADIUS*cos(theta);
arc_y = RADIUS*sin(theta);

tmp_bd  =[0 0; arc_x arc_y; 0 0];

%currdeg = pos(3);
currdeg = 0;
c = cos(currdeg*pi/180);
s = sin(currdeg*pi/180);
rotaion = [c -s;s c]';
tmp_bd = tmp_bd*rotaion;


data = [5 0; 2 10; 25 2];
hold on 
fill(tmp_bd(:,1),tmp_bd(:,2),'r')
scatter(data(:,1),data(:,2))
axis equal

[in,on] = inpolygon(data(:,1), data(:,2), tmp_bd(:, 1), tmp_bd(:, 2));


