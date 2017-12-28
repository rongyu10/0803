function bd = get_sidepoint(pos, H)

x0 = pos(1);
y0 = pos(2);
deg = pos(3);
c = cos(deg*pi/180);
s = sin(deg*pi/180);

x1 = x0 + H*s;
y1 = y0 - H*c;

x2 = x0 - H*s;
y2 = y0 + H*c;


bd = [x1 y1 ; x2 y2];
