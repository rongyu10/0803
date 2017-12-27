function risk_othercars = predict_risk_to_otherCars(mycar, othercars)

MIN_DIST_Y     = 25*10^3;
v_free         = 20*10^3;
%----
MIN_DIST_FRONT = 10*10^3;
%----
%--- GET following cars ID -------------
[idx_frontL,idx_frontR,idx_rearL,idx_rearR] = get_neighboursCars_tollplaza(mycar, othercars);
%---------------------------------------

%--- predict risk to rear-left car -
risk_othercars.rearL = 0;
if (idx_rearL ~=0)
    risk_othercars.rearL = judge_risk_to_followingcar(mycar,othercars,idx_rearL, 'left', MIN_DIST_Y, v_free);
end    
%-----------------------------------

%--- predict risk to rear-right car-
risk_othercars.rearR = 0;
if (idx_rearR ~=0)
    risk_othercars.rearR = judge_risk_to_followingcar(mycar,othercars,idx_rearR, 'right', MIN_DIST_Y, v_free);
end    
%-----------------------------------

%--- predict risk to front-left car -
risk_othercars.frontL = 0;
if (idx_frontL ~=0)
    risk_othercars.frontL = judge_risk_to_frontcar(mycar,othercars,idx_frontL, MIN_DIST_FRONT);
end
%-----------------------------------

%--- predict risk to front-left car -
risk_othercars.frontR = 0;
if (idx_frontR ~=0)
    risk_othercars.frontR = judge_risk_to_frontcar(mycar,othercars,idx_frontR, MIN_DIST_FRONT);
end
%-----------------------------------

%--- Debug -------------------------
%fprintf(1, 'RISK F-LEFT:%d, F-RIGHT:%d \n',risk_othercars.frontL,risk_othercars.frontR);
%fprintf(1, 'RISK R-LEFT:%d, R-RIGHT:%d \n',risk_othercars.rearL,risk_othercars.rearR);
%-----------------------------------

return
end

function risk_rear = judge_risk_to_followingcar(mycar, othercars, idx_rear, Left_or_Right,minimun_distance, v_free)
%  Judging riks to rear-left/rear-right car
%
    %- mycar---
    pos_mycar   = mycar.pos(1:2);
    %theta_mycar = mycar.pos(3);
    %----------
    %- othercar--
    pos_othercar   = othercars.car{idx_rear}.pos(1:2);
    theta_othercar = othercars.car{idx_rear}.pos(3);
    %------------
    %- calc geometric data-----
    vec_othercar_to_mycar = pos_mycar - pos_othercar;
    vec_othercar          = get_directionVector(pos_othercar, theta_othercar) - pos_othercar;
    dist  = norm(vec_othercar_to_mycar);

    vec_othercar_to_mycar_m = [vec_othercar_to_mycar 0];
    vec_othercar_m          = [vec_othercar 0];
    crossProd_tmp      = cross(vec_othercar_to_mycar_m, vec_othercar_m);
    crossProd          = crossProd_tmp(3);

    innerProd = dot(vec_othercar_to_mycar, vec_othercar);
    theta = acos(innerProd/dist);
    %---------------------------
    
%------------------    
if (strcmp(Left_or_Right,'left')) % for rear-left car
    if crossProd < 0 % othercar is NOT heading to egocar direction
       risk_rear = 0;
    elseif (crossProd >= 0)&& (theta > (pi/2)) % othercar is NOT heading to egocar direction
       risk_rear = 0;           
    else
       dist_X = dist*cos(theta); % longitudinal distance between egocar and othercar
       dist_Y = dist*sin(theta); % lateral distance between egocar and othercar
          if (dist_Y < minimun_distance) 
              [~,angry] = predictAccel_by_ACC_tollplaza(v_free, mycar, othercars.car{idx_rear}, dist_X); % evaluate accel by othercar
              if angry > 0
                  risk_rear = 1;
              else
                  risk_rear = 0;
              end
          else
             risk_rear = 0;
          end
    end
elseif  (strcmp(Left_or_Right,'right')) % for rear-right car
    if crossProd > 0 % othercar is NOT heading to egocar direction
       risk_rear = 0;
    elseif (crossProd <= 0)&& (theta > (pi/2)) % othercar is NOT heading to egocar direction
       risk_rear = 0;           
    else
       dist_X = dist*cos(theta); % longitudinal distance between egocar and othercar
       dist_Y = dist*sin(theta); % lateral distance between egocar and othercar
          if (dist_Y < minimun_distance)
              [~,angry] = predictAccel_by_ACC_tollplaza(v_free, mycar, othercars.car{idx_rear}, dist_X); % evaluate accel by othercar
              if angry > 0
                  risk_rear = 1;
              else
                  risk_rear = 0;
              end
          else
             risk_rear = 0;
          end
    end    
    
else
   error('wrong Input in predict_risk_of_followingCars') 
end 
%------------------        
    
end

function risk_front =  judge_risk_to_frontcar(mycar, othercars, idx_front,minimun_distance)
%   distance between ego car and othercars
%   if distance <  minimun_distance : risk = 1
%   if distance >= minimun_distance : risk = 0

    risk_front  = 0;
    
    pos_mycar   = mycar.pos(1:2);
    %theta_mycar = mycar.pos(3);
    
    pos_othercar   = othercars.car{idx_front}.pos(1:2);
    %theta_othercar = othercars.car{idx_front}.pos(3);
    
    vec_othercar_to_mycar = pos_mycar - pos_othercar;
    %vec_othercar          = get_directionVector(pos_othercar, theta_othercar) - pos_othercar;
    dist  = norm(vec_othercar_to_mycar);

    if (dist < minimun_distance)
        risk_front  = 1;
    end
    
end

function vec_direction = get_directionVector(pos, theta)
%  get unit vector pointing to theta direction

unitVec = [1 0];
c    = cos(theta*pi/180);
s    = sin(theta*pi/180);
rotation = [c -s; s c]';

vec_tmp = unitVec*rotation;
vec_direction =vec_tmp + pos;

end
