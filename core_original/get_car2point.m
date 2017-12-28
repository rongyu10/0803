function bd = get_car2point(pos, W, H)

x0 = pos(1);
y0 = pos(2);
deg = pos(3);
c = cos(deg*pi/180);
s = sin(deg*pi/180);


x1 = x0 + W/2*c;
y1 = y0 + W/2*s;

x2 = x0 - W/2*c;
y2 = y0 - W/2*s;



bd = [x1 y1 ; x2 y2];