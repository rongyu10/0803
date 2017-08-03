function othercars = intelligentDriverModel_tmp(othercars,mycar,myinfo,track,sim)

%{
%---- Settings for intelligent driver model --------------
v0    = 12500;     % 5000 desired_velocity[mm/s]
T     = 1.5;       % 1.6  safetime_headway[s]
a_max = 1000;      % 730  maximum acceleration [mm/s^2]
b_max = 3000;      % 1670 confotable decceletion  [mm/s^2]
[car_size, ~] = get_carsize(); % length of car [mm]
delta = 4.0;       % 4.0 acceleration exponent [-]
s0    = 2000;      % 2000 linear jam distance [mm]
%s1    = 3000;     % 3000  non-linear jam distance [mm]
%--------------------------------------------------------
%}
LANE_interaction = 2;
ycoord_interaction = 0; % -450 [mm]
%--------------------------------------------------------

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

%---- X方向（進行方向）座標でソーティング-------------------------
if flag_interaction
    [traffic.order, traffic.mycar_idx] = get_allcars_info(othercars,mycar); % mycarがレーンチェンジした場合
else
    [traffic.order, traffic.mycar_idx] = get_othercars_info(othercars);
end
%---------------------------------------------------------------

ncars         = length(traffic.order);
traffic.n     = ncars;
traffic.car   = cell(ncars, 1);
trackLength   = track.xmax - track.xmin;

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

%---- IDMによる速度の更新(mycarは速度を更新しない)---------
for i = 1:ncars  
    if i ~= traffic.mycar_idx
       % for yielding case
       if isfield(othercars.car{traffic.order(i)},'yield')
           if (othercars.car{traffic.order(i)}.yield ==1)
               continue
           end
       end
       %---- Settings for intelligent driver model --------------
       v0       = othercars.car{traffic.order(i)}.IDM.v0;
       T        = othercars.car{traffic.order(i)}.IDM.T;
       a_max    = othercars.car{traffic.order(i)}.IDM.a_max;
       b_max    = othercars.car{traffic.order(i)}.IDM.b_max;
       car_size = othercars.car{traffic.order(i)}.W;
       delta    = othercars.car{traffic.order(i)}.IDM.delta;
       s0       = othercars.car{traffic.order(i)}.IDM.s0;
       %--------------------------------------------------------  
       v = traffic.car{i}.vel(1);
       x = traffic.car{i}.pos(1);  
          if i ==1
             % periodic boundary condition
             v_1 = traffic.car{ncars}.vel(1);
             x_1 = traffic.car{ncars}.pos(1);
             deltaV  = v - v_1;
             s_alpha = x_1 - x + trackLength - car_size;
          else
             v_1 = traffic.car{i-1}.vel(1);
             x_1 = traffic.car{i-1}.pos(1);
             deltaV  = v - v_1;
             s_alpha = x_1 - x - car_size;       
          end
    
        s_star = s0 + max(0.0, v*T + (v*deltaV)/(2.0*sqrt(a_max*b_max)));
        relativeVelocity = v/v0;
        relativeGap      = s_star/s_alpha;
        acceleration     = a_max*(1.0 - relativeVelocity.^delta - relativeGap.^2);
        v_new = v + acceleration * sim.T;
        othercars.car{traffic.order(i)}.vel(1) = max(0.0,v_new); % 速度は正
        othercars.car{traffic.order(i)}.accele = acceleration;

        if (acceleration < -3000)&&(acceleration > -7000)
            othercars.car{traffic.order(i)}.IDM.angry = 1;
        elseif (acceleration <= -7000)
            othercars.car{traffic.order(i)}.IDM.angry = 2;
        else
            othercars.car{traffic.order(i)}.IDM.angry = 0;            
        end
    end
end
%-------------------------------------------------------

end


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


function [xorder, mycar_idx] = get_allcars_info(othercars,mycar)

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

%--- my car position------
xcoord = [xcoord,mycar.pos(1)];
id = [id, nr_cars +1];
%--------------------------

[xtmp,itmp] = sort(xcoord,'descend');
xorder = id(itmp);
mycar_idx = find(xtmp==mycar.pos(1));

end
