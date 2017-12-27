function othercars = intelligentDriverModel4(othercars,mycar,myinfo,road,sim)
% This IDM function is designed for Toll Plaza simulation

%{
%---- Settings for intelligent driver model --------------
v0_norm = 20000;   % 5000 desired_velocity[mm/s]
v0_gate = 7000;    % 5000 desired_velocity[mm/s]
T     = 1.5;       % 1.6  safetime_headway[s]
a_max = 1000;      % 730  maximum acceleration [mm/s^2]
b_max = 3000;      % 1670 confotable decceletion  [mm/s^2]
[car_size, ~] = get_carsize(); % length of car [mm]
delta = 4.0;       % 4.0 acceleration exponent [-]
s0    = 2000;      % 2000 linear jam distance [mm]
%s1    = 3000;     % 3000  non-linear jam distance [mm]
%--------------------------------------------------------
%}
v0_gate = 7000;    % 5000 desired_velocity[mm/s]


%---- X方向（進行方向）座標でソーティング-------------------------
 [traffic.order, traffic.mycar_idx] = get_allcars_info(othercars,mycar);
 %---------------------------------------------------------------
forwardCarID = get_forwardCarID(traffic.order, traffic.mycar_idx, othercars, mycar);
ncars         = length(traffic.order);
traffic.n     = ncars;
traffic.car   = cell(ncars, 1);
trackLength   = road.xmax - road.xmin;

%---- IDMによる速度の更新(mycarとXcoord<0の車両の速度を更新しない)---------
idx_mycar = traffic.order(traffic.mycar_idx);
for i = 1:ncars
    idx      = traffic.order(i);
    iforward = forwardCarID(i);
    if idx ~= idx_mycar
       %---- Settings for intelligent driver model --------------
       v0_norm  = othercars.car{traffic.order(i)}.IDM.v0;
       T        = othercars.car{traffic.order(i)}.IDM.T;
       a_max    = othercars.car{traffic.order(i)}.IDM.a_max;
       b_max    = othercars.car{traffic.order(i)}.IDM.b_max;
       car_size = othercars.car{traffic.order(i)}.W;
       delta    = othercars.car{traffic.order(i)}.IDM.delta;
       s0       = othercars.car{traffic.order(i)}.IDM.s0;
       %--------------------------------------------------------         
       v = othercars.car{idx}.vel(1);
       x = othercars.car{idx}.pos(1:2);
       theta = othercars.car{idx}.pos(3);
          %------------------------
          if iforward == 0
             % periodic boundary condition
             %tmp_forward  = traffic.order(ncars);
             %v_1 = othercars.car{tmp_forward}.vel(1);
             %x_1 = othercars.car{tmp_forward}.pos(1);
             %deltaV  = v - v_1;
             %s_alpha = x_1 - x + trackLength - car_size;
             
          elseif iforward == idx_mycar
             velo_1  = mycar.vel(1);
             x_1     = mycar.pos(1:2);
             theta_1 = mycar.pos(3);
             v_1     = get_speed_in_headingDirection(theta, velo_1, theta_1);
             deltaV  = v - v_1;
             interval = norm( x_1 - x );  % direct distance( must be changed in case of small-R curve)
             s_alpha  = interval - car_size;
          else
             velo_1 = othercars.car{iforward}.vel(1);
             x_1 = othercars.car{iforward}.pos(1:2);
             theta_1 = othercars.car{iforward}.pos(3);
             v_1     = get_speed_in_headingDirection(theta, velo_1, theta_1);
             deltaV  = v - v_1;
             interval = norm( x_1 - x );  % direct distance( must be changed in case of small-R curve)
             s_alpha = interval - car_size;       
          end
          %------------------------
          if iforward ~= 0
            s_star = s0 + max(0.0, v*T + (v*deltaV)/(2.0*sqrt(a_max*b_max)));
            relativeGap      = s_star/s_alpha;
          else
            relativeGap      = 0.0;
          end
          %------------------------
          
          %----Speed Limit at the gate ---------
          if (x(1) > 275*10^3)
             v0 = v0_gate;
          else
             v0 = v0_norm;              
          end
          %-------------------------------------
          
          if (x(1) > 0) % update speed for cars inside the simulator road
            relativeVelocity = v/v0;
            acceleration     = a_max*(1.0 - relativeVelocity.^delta - relativeGap.^2);
            v_new = v + acceleration * sim.T;
            othercars.car{idx}.vel(1) = max(0.0,v_new); % speed must be positive
            othercars.car{idx}.accele = acceleration;
            
            %---- drivers reaction ---------------------
            if (acceleration < -3000)&&(acceleration > -7000)
               othercars.car{idx}.IDM.angry = 1;
            elseif (acceleration <= -7000)
               othercars.car{idx}.IDM.angry = 2;
            else
               othercars.car{idx}.IDM.angry = 0;
            end
            %-------------------------------------------
          end
          
    end
end
%-------------------------------------------------------


end

function [xorder, mycar_idx] = get_allcars_info(othercars,mycar)
% xorder: ID number in x-coord descending order

nr_cars = othercars.n;

allpos = get_allPosition(othercars, mycar);
xcoord = allpos(:,1);
id = 1:nr_cars+1;

[xtmp,itmp] = sort(xcoord,'descend');
xorder = id(itmp);
mycar_idx = find(itmp==(nr_cars+1));
%mycar_idx = find(xtmp==mycar.pos(1));

end

function forwardCarID = get_forwardCarID(xorder, mycar_idx,othercars, mycar)

persistent first_flag h
if isempty(first_flag)
    first_flag = true;
end

ncars = length(xorder);
forwardCarID = zeros(1,ncars);

%allpos = get_allPosition(othercars, mycar);
allBD = get_allBoundary(othercars, mycar);

for i = 1:ncars
    if i == mycar_idx
        pos = mycar.pos;
    else
        pos = othercars.car{xorder(i)}.pos;
    end
    
    if i ==1 % case of leading car
        forwardCarID(i) = 0;
    elseif pos(1)< 0 % outside of the simulater (X cood is negative)
        forwardCarID(i) = 0;
    else

       if ~isfield(othercars,'traj')
          %searchbd = searchArea_ArcType(pos);
          searchbd = searchArea_RectType(pos);
       else
            if (i~=mycar_idx)&&(isfield(othercars,'info'))&&...
               (isfield(othercars.info{xorder(i)},'type'))&&(strcmp(othercars.info{xorder(i)}.type,'plaza')) % inside the plaza
                searchbd = searchArea_TrajectoryType(pos,othercars.traj{xorder(i)});
            else
                %searchbd = searchArea_ArcType(pos);
                searchbd = searchArea_RectType(pos);
            end
       end
       
       %--- for debug-----
%        if (xorder(i)==6)
%            if first_flag == true
%               h.bd = plot(searchbd(:,1),searchbd(:,2),'-','Color','b','LineWidth',2);
%               first_flag = false; 
%            else
%                h.bd.XData = searchbd(:,1)';
%                h.bd.YData = searchbd(:,2)';
%            end
%        end
       %-------------------
                  
       %iforward = get_forwardCarIndex(i, xorder, allBD, searchbd);
       iforward = get_forwardCarIndex_modified(i, xorder, allBD, searchbd);   % Fastest
       %iforward = get_forwardCarIndex_by_InterX(i, xorder, allBD,searchbd);  % Slowest.... 

       if iforward==0
          forwardCarID(i) = 0;
       else
          forwardCarID(i) = xorder(iforward);
       end
    end
end


end

%----------------------------------------------------------------------
function iforward = get_forwardCarIndex(i_mynum, xorder, allBD, searchbd)
%

%--- Leading car case---
if i_mynum ==1
    iforward = 0;
    return
end
%-----------------------

flag    = zeros(i_mynum-1, 1);
for i = 1:i_mynum-1
    tmp_otherBD = allBD{xorder(i),1};
    otherBD = tmp_otherBD';    
       %----    
       P = inpolygon(otherBD(:,1), otherBD(:,2), searchbd(:, 1), searchbd(:, 2));
       if sum(P)==0
           flag(i) = 0;
       else
           flag(i) = 1;
       end
       %----
end

tmp = sum(flag);
if tmp==0
   iforward = 0;
   return
else
   iforward = find(flag,1,'last');
   return
end


end

function iforward = get_forwardCarIndex_modified(i_mynum, xorder, allBD, searchbd)
%  Faster version of get_forwardCarIndex

%--- Leading car case---
if i_mynum ==1
    iforward = 0;
    return
end
%-----------------------

%-- number of data set of each car ---
NSET = 7;
%-------------------------------------

flag       = zeros(i_mynum-1, 1);
otherBDall = zeros((i_mynum-1)*NSET,2);

for i = 1:i_mynum-1
    tmp_otherBD = allBD{xorder(i),1};
    ista  = (i-1)*NSET +1;
    iend  = i*NSET;
    otherBDall(ista:iend,:) = tmp_otherBD';
end

%---- 
P  = inpolygon(otherBDall(:,1), otherBDall(:,2), searchbd(:, 1), searchbd(:, 2));
%----

%--------
for i = 1:i_mynum-1
    ista  = (i-1)*NSET+1;
    iend  = i*NSET;
    tmp_P = P(ista:iend);
    
    if sum(tmp_P)==0
      flag(i) = 0;
    else
      flag(i) = 1;
    end
end
%-------

tmp = sum(flag);
if tmp==0
   iforward = 0;
   return
else
   iforward = find(flag,1,'last');
   return
end


end

%{
function iforward = get_forwardCarIndex_by_InterX(i_mynum, xorder, allBD, searchbd)


flag    = zeros(i_mynum-1, 1);
for i = 1:i_mynum-1
    otherBD = allBD{xorder(i),1};
    %----    
    P = InterX(searchbd', otherBD);
    if isempty(P)
        flag(i) = 0;
    else
        flag(i) = 1;
    end
    %----
end

tmp = sum(flag);
if tmp==0
   iforward = 0;
   return
else
   
   iforward = find(flag,1,'last');
   return
end

end
%}
%----------------------------------------------------------------------

%----------------------------------------------------------------------
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


function allBD = get_allBoundary(othercars, mycar)

nr_cars = othercars.n;
allBD  = cell(nr_cars+1,1);

for i=1:nr_cars
   allBD{i,1} = get_carLineData(othercars.car{i});

end
%--- my car position------
 allBD{nr_cars+1,1} = get_carLineData(mycar);
%--------------------------

end


function lineData = get_carLineData(car)

lineData = [car.bd(1,1),car.bd(2,1),car.bd(4,1),car.bd(5,1),car.bd(7,1),car.bd(8,1);... 
            car.bd(1,2),car.bd(2,2),car.bd(4,2),car.bd(5,2),car.bd(7,2),car.bd(8,2)];

% -- added car center point----
lineData(1,7) = car.pos(1);
lineData(2,7) = car.pos(2);
%-----------------------------
end

%----------------------------------------------------------------------
%{
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
%}

%----------------------------------------------------------------------
function bd = searchArea_RectType(pos)
%-- Area Setting----
Length = 50*10^3;     % 30 m
H      = 3500; % 2500  car width
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


%----------------------------------------------------------------------
%    searching box along the trajectory 
function bd = searchArea_TrajectoryType(pos,routeData)
%-- Area Setting----
Length  = 50*10^3;     % 30 m
ITVL    =  6;
H       = 4500; % 3500  car width
%--------------------

lengthData = linspace(0,Length, ITVL);
posData      = zeros(ITVL,3);

%---- points along the trajectory-----
posData(1,:) = pos;
curr_idx = current_indexX(pos,routeData);
for i = 2:ITVL
    [posData(i,:), ~] = get_next_position(pos, routeData, curr_idx, lengthData(i));
end
%-------------------------------------

bd = zeros(2*ITVL+1,2);

%--- boundary box along the trajectory -----
uniVec = [H/2, 0];
for i = 1:ITVL
  tmp_pos = posData(i,1:2);
  
  deg = posData(i,3)+90;
  c   = cos(deg*pi/180);
  s   = sin(deg*pi/180);
  rotaion = [c -s;s c]';
  transVec= uniVec*rotaion;
  bd(i,:)            = tmp_pos + transVec;
  bd((2*ITVL+1)-i,:) = tmp_pos - transVec;     
end
bd(end,:) = bd(1,:);
%-------------------------------------------
end

function idx_x = current_indexX(pos,routeData)

data    = routeData.route;
n_data  = size(data,1);
curr_x  = pos(1);

idx_x = 0;
for i=1:n_data-1
  idx_x = i;
  if curr_x < data(i,1)
     break
  end
end

end

function [next_pos, next_idx_x] = get_next_position(pos, routeData, curr_idx, dlength)
% get next position at a distance of 'dlength' along the trajectory 

 data    = routeData.route;
 n_data  = size(data,1);
 seg_len = routeData.seg_len;
 seg_ang = routeData.seg_ang;
 %------------------
 next_pos = zeros(1,3);
 %----------------------------------------------
 diffx   = data(curr_idx+1, 1) - pos(1);
 diffy   = data(curr_idx+1, 2) - pos(2);
 length1 = sqrt(diffx*diffx + diffy*diffy);
 if dlength < length1
    radian = seg_ang(curr_idx,1)*pi/180;
    next_idx_x = curr_idx;
    next_pos(1) = pos(1) + dlength*cos(radian);
    next_pos(2) = pos(2) + dlength*sin(radian);    
    next_pos(3) = seg_ang(curr_idx,1);            
    return
 end
 if (curr_idx ==(n_data-1))
    next_idx_x  = n_data;
    next_pos(1) = data(end,1) + dlength - length1;
    next_pos(2) = data(end,2);
    next_pos(3) = 0.0;
    return
 end
 %---------------------------------------------
 
 tmp_length = length1;
 for i = curr_idx+1:n_data-1
     next_idx_x = i;
     tmp_length = tmp_length + seg_len(i);
     if dlength < tmp_length
        break
     end
 end
 
 length2 = tmp_length - dlength;
 
 radian = seg_ang(next_idx_x,1)*pi/180;
 next_pos(1) = data(next_idx_x+1, 1) - length2 * cos(radian);
 next_pos(2) = data(next_idx_x+1, 2) - length2 * sin(radian);    
 next_pos(3) = seg_ang(next_idx_x,1);
 return
end
%---------------------------------------------
% Velocity conponent considering direction
function v_1_heading = get_speed_in_headingDirection(theta, velo_1, theta_1)

%------------
unitVec = [1 0];
c    = cos(theta*pi/180);
s    = sin(theta*pi/180);
rotation = [c -s; s c]';
vec_dire = unitVec*rotation;
%------------
c    = cos(theta_1*pi/180);
s    = sin(theta_1*pi/180);
v_1  = velo_1*[c s];
%------------

v_1_heading = dot(vec_dire, v_1);


end
%---------------------------------------------
