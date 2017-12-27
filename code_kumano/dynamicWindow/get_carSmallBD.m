function bd = get_carSmallBD(carpos)
%

pos   = carpos(1:2);

theta = linspace(0,2*pi,20)';
ndata = length(theta);
trans_pos = pos(ones(ndata,1),:);

a = 3000;
b = 1500;
x = a*cos(theta);
y = b*sin(theta);

local_bd = [x ,y];

bd = local_bd + trans_pos;

%scatter(bd(:,1),bd(:,2));

end