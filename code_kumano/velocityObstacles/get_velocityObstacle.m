function [vo,vo_globalcoord] = get_velocityObstacle(mycar, person,varargin)

FLAG_DEBUG = false;

if (nargin == 3)
   maxspeed = varargin{1};
else
   maxspeed = 15*10^3;
end

%--- object A (mycar)---------
pA   = mycar.pos(1:2);
angA = mycar.pos(3);
vA   = mycar.vel(1)*[cos(toRadian(angA)), sin(toRadian(angA))];
rA   = mycar.H*0.5; 
%rA   = mycar.W*0.5;
%rA   = (mycar.W+mycar.H)*0.25;
%-----------------------------

%--- object B (pedestrian)----
pB   = person.pos(1:2);
angB = person.pos(3);
vB   = person.vel(1)*[cos(toRadian(angB)), sin(toRadian(angB))];
rB   = person.D*0.5;
%-----------------------------

trans_vB_vA = 0.5*(vA+vB);
dist_AB      = norm(pB-pA,2);
theta_AB     = atan2( (pB(2)-pA(2)), (pB(1)-pA(1)) );
%{
theta_AB     = atan( (pB(2)-pA(2))/(pB(1)-pA(1)) );
if (pB(1)-pA(1))<0
  theta_AB = theta_AB + pi;
end
%}

rAB  = rA + rB;

if dist_AB < rAB   
   dist_AB = rAB;
end

openTheta = asin(rAB/dist_AB);

thetaLeft  = theta_AB + openTheta;
thetaRight = theta_AB - openTheta;

voLeft  = maxspeed*[cos(thetaLeft), sin(thetaLeft)]  + trans_vB_vA;
voRight = maxspeed*[cos(thetaRight),sin(thetaRight)] + trans_vB_vA;

%-- Reciprocal Velocity OBstacle in velocity space -----------
vo = [trans_vB_vA(1),voRight(1),voLeft(1),trans_vB_vA(1);...
      trans_vB_vA(2),voRight(2),voLeft(2),trans_vB_vA(2)];
%--------------------------------------------------------------

%-- Reciprocal Velocity OBstacle in global coordinate (this is used for plot)-----------
trans2_vB_vA = trans_vB_vA + pA;
voLeft2  = maxspeed*[cos(thetaLeft), sin(thetaLeft)]  + trans2_vB_vA;
voRight2 = maxspeed*[cos(thetaRight),sin(thetaRight)] + trans2_vB_vA;

vo_globalcoord = [trans2_vB_vA(1),voRight2(1),voLeft2(1),trans2_vB_vA(1);...
                  trans2_vB_vA(2),voRight2(2),voLeft2(2),trans2_vB_vA(2)];
%------------------------------------------------------------------------------------

%{
if FLAG_DEBUG
%-----for plotting Velocity Obstacles----------------------------
voLeft_old  = maxspeed*[cos(thetaLeft), sin(thetaLeft)];
voRight_old = maxspeed*[cos(thetaRight),sin(thetaRight)];

vo_old = [0.0,voRight_old(1),voLeft_old(1),0.0;...
          0.0,voRight_old(2),voLeft_old(2),0.0];
%---      
dv = linspace(0,2*pi,30);
one = ones(1,length(dv));
cir = rAB*[cos(dv); sin(dv)];
cir(1,:) = cir(1,:) +one*(pB(1)-pA(1));
cir(2,:) = cir(2,:) +one*(pB(2)-pA(2));      
%------      

hold on;
fill(vo_old(1,:),vo_old(2,:),'c')
fill(vo(1,:),vo(2,:),'b')
fill(vo_globalcoord (1,:),vo_globalcoord (2,:),'r')
plot(cir(1,:),cir(2,:),'k')
scatter(trans_vB_vA(1),trans_vB_vA(2))
scatter(0,0,'r','p')
scatter(pB(1)-pA(1),pB(2)-pA(2),'r','+')
%axis([-maxspeed  maxspeed  -maxspeed  maxspeed ])
axis square
%-------------------------------------------------
end
%}

end


function radian = toRadian(degree)
% degree to radian
radian = degree/180*pi;
end
