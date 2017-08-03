%-- test fill
clear
%{
theta = 0:0.1:2*pi;
one = ones(1,length(theta));

x1 = cos(theta);
y1 = sin(theta);

x2 = x1 + one*5;
y2 = y1 + one*5;

x  = [x1,x2];
y  = [y1,y2];

h = patch(x,y,'b');
%}

V=[0 0;0 1;1 0;1 1;2 0];
C={[1 2 3];[3 2 4 5]};
FV.vertices=V;
FV.faces=C{1};
patch(FV);
FV.faces=C{2};
patch(FV,'facecolor','blue');



