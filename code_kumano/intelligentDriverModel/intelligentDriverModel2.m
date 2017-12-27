function othercars = intelligentDriverModel2(othercars,mycar,myinfo,road,sim)

%---- Settings for intelligent driver model --------------
v0    = 35000;      % 5000 desired_velocity[mm/s]
T     = 1.2;       % 1.6  safetime_headway[s]
a_max = 4000;      % 730  maximum acceleration [mm/s^2]
b_max = 1500;      % 1670 confotable decceletion  [mm/s^2]
[car_size, ~] = get_carsize(); % length of car [mm]
delta = 4.0;       % 4.0 acceleration exponent [-]
s0    = 2000;      % 2000 linear jam distance [mm]
%s1    = 3000;     % 3000  non-linear jam distance [mm]
%--------------------------------------------------------
%LANE_interaction = 2;
%ycoord_interaction = -450; % -125 [mm]
%--------------------------------------------------------

%{
%----mycarが白線をycoord_interaction越えたらレーンチェンジしたものとする
flag_interaction = false;
if myinfo.lane_idx == LANE_interaction
   flag_interaction = true;
elseif mycar.bd(1,2) < ycoord_interaction % right rear corner of my car
   flag_interaction = true;
elseif mycar.bd(2,2) < ycoord_interaction % right front corner of my car
   flag_interaction = true;   
end
%----------------------------------------------------------------
%}

%---- X方向（進行方向）座標でソーティング-------------------------
 [traffic.order, traffic.mycar_idx] = get_allcars_info(othercars,mycar);

 %---------------------------------------------------------------
forwardCarID = get_forwardCarID(traffic.order, traffic.mycar_idx, othercars, mycar);
 
ncars         = length(traffic.order);
traffic.n     = ncars;
traffic.car   = cell(ncars, 1);
trackLength   = road.xmax - road.xmin;

%{
for i = 1:ncars
    if i == traffic.mycar_idx
        traffic.car{i}.pos = mycar.pos;
        traffic.car{i}.vel = mycar.vel; 
        traffic.car{i}.W   = mycar.W;
        traffic.car{i}.H   = mycar.H;        
    else
        traffic.car{i}.pos = othercars.car{traffic.order(i)}.pos;
        traffic.car{i}.vel = othercars.car{traffic.order(i)}.vel; 
        traffic.car{i}.W   = othercars.car{traffic.order(i)}.W;
        traffic.car{i}.H   = othercars.car{traffic.order(i)}.H;
    end
end
%}

%---- IDMによる速度の更新(mycarは速度を更新しない)---------
idx_mycar = traffic.order(traffic.mycar_idx);
for i = 1:ncars
    idx      = traffic.order(i);
    iforward = forwardCarID(i);
    if idx ~= idx_mycar
       v = othercars.car{idx}.vel(1);
       x = othercars.car{idx}.pos(1);  
          if iforward == 0
             % periodic boundary condition
             %tmp_forward  = traffic.order(ncars);
             %v_1 = othercars.car{tmp_forward}.vel(1);
             %x_1 = othercars.car{tmp_forward}.pos(1);
             %deltaV  = v - v_1;
             %s_alpha = x_1 - x + trackLength - car_size;
             
          elseif iforward == idx_mycar
             v_1 = mycar.vel(1);
             x_1 = mycar.pos(1);
             deltaV  = v - v_1;
             s_alpha = x_1 - x - car_size;
          else
             v_1 = othercars.car{iforward}.vel(1);
             x_1 = othercars.car{iforward}.pos(1);
             deltaV  = v - v_1;
             s_alpha = x_1 - x - car_size;       
          end
          
          if iforward ~= 0
            s_star = s0 + max(0.0, v*T + (v*deltaV)/(2.0*sqrt(a_max*b_max)));
            relativeGap      = s_star/s_alpha;
          else
            relativeGap      = 0.0;
          end
          
          %if x > 275000
          %    v0 = 10000;
          %end          
          relativeVelocity = v/v0;
          acceleration     = a_max*(1.0 - relativeVelocity.^delta - relativeGap.^2);
          v_new = v + acceleration * sim.T;
          othercars.car{traffic.order(i)}.vel(1) = max(0.0,v_new); % 速度は正
    end
end
%-------------------------------------------------------





end

%{
function [xorder, mycar_idx] = get_othercars_info(othercars)

nr_cars = othercars.n;
id = [];
xcoord = [];

for i=1:nr_cars
    if strcmp(othercars.car{i,1}.ctrlmode,'normal')
        id =[id,i];
        x = othercars.car{i,1}.pos(1);
        xcoord =[xcoord,x];
    end
end

[~,itmp] = sort(xcoord,'descend');
xorder = id(itmp);
mycar_idx = 0;

end
%}

function [xorder, mycar_idx] = get_allcars_info(othercars,mycar)

nr_cars = othercars.n;

allpos = get_allPosition(othercars, mycar);
xcoord = allpos(:,1);
id = 1:nr_cars+1;

[xtmp,itmp] = sort(xcoord,'descend');
xorder = id(itmp);
mycar_idx = find(xtmp==mycar.pos(1));

end

function forwardCarID = get_forwardCarID(xorder, mycar_idx,othercars, mycar)

ncars = length(xorder);
forwardCarID = zeros(1,ncars);

allpos = get_allPosition(othercars, mycar);

for i = 1:ncars
    if i == mycar_idx
        pos = mycar.pos;
    else
        pos = othercars.car{xorder(i)}.pos;
    end

    if i ==1
        forwardCarID(i) = 0;
    else
       %searchbd = searchArea_ArcType(pos);
       searchbd = searchArea_RectType(pos);
       if i<=11
            idx = xorder(1:i-1);
       else
            idx = xorder(i-10:i-1);
       end
           
       otherpos = allpos(idx,:);
       iforward = get_forwardCarIndex( otherpos, searchbd );
       if iforward==0
          forwardCarID(i) = 0;
       else
          forwardCarID(i) = idx(iforward);
       end
    end
end


end

function iforward = get_forwardCarIndex( otherpos, searchbd )

in = inpolygon(otherpos(:,1), otherpos(:,2), searchbd(:, 1), searchbd(:, 2));

tmp = sum(in);
if tmp==0
   iforward = 0;
   return
else
   iforward = find(in,1,'last');
   return
end

end



function allpos = get_allPosition(othercars, mycar)

nr_cars = othercars.n;
allpos  = zeros(nr_cars+1,2);

for i=1:nr_cars
    allpos(i,1:2) = othercars.car{i}.pos(1:2);
end
%--- my car position------
allpos(nr_cars+1,1:2) = mycar.pos(1:2);
%--------------------------

end

function bd = searchArea_ArcType(pos)
%-- Area Setting----
ANGLE  = 16*(pi/180); % 16 degree
%ANGLE  = 20*(pi/180); % 16 degree
RADIUS = 50*10^3;     % 30 m
%--------------------
theta  = linspace(-0.5*ANGLE, 0.5*ANGLE, 6)';

arc_x = RADIUS*cos(theta);
arc_y = RADIUS*sin(theta);

tmp_bd  = [0 0; arc_x arc_y; 0 0];

%--rotation-----
currdeg = pos(3);
c = cos(currdeg*pi/180);
s = sin(currdeg*pi/180);
rotaion = [c -s;s c]';
tmp_bd = tmp_bd*rotaion;
%----------------
nsize = size(tmp_bd,1);
bd = tmp_bd + repmat(pos(1:2),nsize,1);

end

function bd = searchArea_RectType(pos)
%-- Area Setting----
Length = 50*10^3;     % 30 m
H      = 2500;
W      = 4900;
%--------------------

tmp_bd  = [0 -H/2; Length -H/2; Length  H/2; 0  H/2; 0 -H/2];

%--rotation-----
currdeg = pos(3);
c = cos(currdeg*pi/180);
s = sin(currdeg*pi/180);
rotaion = [c -s;s c]';
tmp_bd = tmp_bd*rotaion;
%----------------
nsize = size(tmp_bd,1);
bd = tmp_bd + repmat(pos(1:2),nsize,1);

end

