function bd = get_carVeryLargeBD(carpos)
%

pos   = carpos(1:2);
angle = carpos(3);
c    = cos(angle*pi/180);
s    = sin(angle*pi/180);
rotation = [c -s; s c]';

%--- ellipse-shepe boundary ----
theta = linspace(0,2*pi,30)';
%ndata = length(theta);
a = 10000; % 7000
b = 2000;  % 2000
x = a*cos(theta);
y = b*sin(theta);
local_bd = [x ,y];
barX   = linspace(-a,a,40)';
ndata2 = length(barX);
barY = zeros(ndata2,1);
bar  = [barX barY];
local_bd = [local_bd;bar];
%-------------------------------
ndata     = size(local_bd,1);
trans_pos = pos(ones(ndata,1),:);

%- rotation & translation ----
bd = local_bd*rotation + trans_pos;
%-----------------------------

%scatter(bd(:,1),bd(:,2));

end


