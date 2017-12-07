function mycar = calculate_velocity_mycar_tollPlaza_IN_1123(mycar, sim, othercars, idm, laneChangePath)

% setting of crossing with othercar-----------------------
MYCAR_AGGRESSIVE_MODE = 2; % mycar agressive mode when crossing with othercar (0:calm, 1:medium, 2:aggressive)

x_start_detecting = 50*10^3;
max_intersect_point = 60*10^3;
max_time_dif_intersection = 2.0; % 
min_time_other_intersection = 1.0; % mycar must yield if othercar arrive intersect point within this value
time_mergin_crossing = 1.0; % time interval of crossing
% --------------------------------------------------------

idx_precedingcar = [];
idx_crossingcar = [];
mycar.acceleration = [];

% if entering the plaza
if mycar.flgPlaza == 0 && mycar.pos(1) > 100*10^3
    mycar.flgPlaza = 1;
    
end

if mycar.flgPlaza == 0 % before entering plaza
    idm.v0 = 15000;
    mycar.pathTranslated = laneChangePath{mycar.goallane, mycar.save.lane_idx};
    
elseif mycar.flgPlaza == 1 % after entering plaza
    pos = predict_pos(mycar.pos, mycar.vel, sim.T);
    mycar.targetDegree = get_tatgetTheta(pos,mycar.pathTranslated);
    mycar.vel(2) = (mycar.targetDegree - mycar.pos(3))/sim.T;
    
    if mycar.pos(1) < 187.5*10^3
        idm.v0 = 15000;

    elseif mycar.pos(1) < 275*10^3
        idm.v0 = 12500;
        
    elseif mycar.pos(1) < 320*10^3
        idm.v0 = 10000;
    end
    
end

% [mycar.squareX, mycar.squareY] = make_detecting_rectangle_lengthfix(mycar, mycar.pos, laneChangePath, mycar.detect_rect_length, mycar.detect_rect_sidewidth);


% estimate crashing othercar and identify approaching othercar in front of mycar----------------------
if mycar.pos(1) > x_start_detecting
    [idx_precedingcar, rel_degree_precedingcar] = detect_preceding_car(othercars, mycar);
    [idx_crossingcar, arr_t_mycar, arr_t_othercar, dist_section_mycar] = detect_crossing_car(othercars, mycar, max_intersect_point);
end

if ~isempty(idx_crossingcar)
    if abs(arr_t_othercar - arr_t_mycar) > max_time_dif_intersection % if over 1(sec) difference between two cars, mycar does not accelerate and decelerate
        A1 = mycar.vel(1)/idm.v0;
        mycar.acceleration = idm.a*(1 - A1^idm.delta);
    else
        if arr_t_othercar < max_time_dif_intersection
            mycar.acceleration = 2*(dist_section_mycar - mycar.vel(1)*(arr_t_othercar + time_mergin_crossing)) / (arr_t_othercar + time_mergin_crossing)^2;
        else
            acceleration_if_front = 2*(dist_section_mycar - mycar.vel(1)*(arr_t_othercar - time_mergin_crossing)) / (arr_t_othercar - time_mergin_crossing)^2;
            acceleration_if_back = 2*(dist_section_mycar - mycar.vel(1)*(arr_t_othercar + time_mergin_crossing)) / (arr_t_othercar + time_mergin_crossing)^2;
            fprintf(1, 'acceleration_if_front = [%d], acceleration_if_back = [%d]\n', acceleration_if_front, acceleration_if_back);
            
            if MYCAR_AGGRESSIVE_MODE == 0
                if acceleration_if_back > -2940
                    mycar.acceleration = acceleration_if_back;
                else
                    mycar.acceleration = acceleration_if_front;
                end
            elseif MYCAR_AGGRESSIVE_MODE == 1
                if abs(acceleration_if_front) < abs(acceleration_if_back)
                    mycar.acceleration = acceleration_if_front;
                else
                    mycar.acceleration = acceleration_if_back;
                end
            else
                if acceleration_if_front < 2940
                    mycar.acceleration = acceleration_if_front;
                else
                    mycar.acceleration = acceleration_if_back;
                end
            end
            
        end
    end
end
    
if ~isempty(idx_precedingcar)
    
    temp_acceleration = calculate_acceleration_IDM_following(mycar, othercars.car{idx_precedingcar}, othercars.car{idx_precedingcar}.pos, idm, rel_degree_precedingcar);
    if ~isempty(mycar.acceleration)
        if temp_acceleration < mycar.acceleration
            mycar.acceleration = temp_acceleration;
        end
    else
        mycar.acceleration = temp_acceleration;
    end
        
end

if isempty(idx_crossingcar) && isempty(idx_precedingcar)
    A1 = mycar.vel(1)/idm.v0;
    mycar.acceleration = idm.a*(1 - A1^idm.delta);
end


mycar.vel(1) = mycar.vel(1) + mycar.acceleration*sim.T;


% --regulate acceleration and velocity ---------------------
if mycar.acceleration > 2940
    mycar.acceleration = 2940;
elseif mycar.acceleration < -2940
    mycar.acceleration = -2940;
end

if ~isempty(idx_precedingcar)
    if mycar.acceleration < -1960
        fprintf(2, 'mycar([%d, %d]) decelerate([%d]) to car [%d] (reldegree = [%d])\n', mycar.pos(1), mycar.pos(2), mycar.acceleration, idx_precedingcar, rel_degree_precedingcar);
    else
        fprintf(1, 'mycar([%d, %d]) decelerate([%d]) to car [%d] (reldegree = [%d])\n', mycar.pos(1), mycar.pos(2), mycar.acceleration, idx_precedingcar, rel_degree_precedingcar);
    end
end

if ~isempty(idx_crossingcar)
    if mycar.acceleration < -1960
        fprintf(2, 'mycar([%d, %d]) decelerate([%d]) to car [%d] (arr_t_mycar=[%d], arr_t_othercar=[%d], dist_section_mycar=[%d])\n', mycar.pos(1), mycar.pos(2), mycar.acceleration, idx_crossingcar, arr_t_mycar, arr_t_othercar, dist_section_mycar);
    else
        fprintf(1, 'mycar([%d, %d]) decelerate([%d]) to car [%d] (arr_t_mycar=[%d], arr_t_othercar=[%d], dist_section_mycar=[%d])\n', mycar.pos(1), mycar.pos(2), mycar.acceleration, idx_crossingcar, arr_t_mycar, arr_t_othercar, dist_section_mycar);
    end
end

if mycar.vel(1) < 0 && mycar.pos(1) < 320 * 10^3
    mycar.vel(1) = 0;
end
% -----------------------------------------------------------

end

function targetDegree = get_tatgetTheta(pos,path)

nData = size(path,1);

dist=bsxfun(@hypot,path(:,1)-pos(1),path(:,2)-pos(2));
[~,idx]=min(dist);

if idx~=nData
    vx= path(idx+1,1)-path(idx,1);
    vy= path(idx+1,2)-path(idx,2);
else
    vx= path(idx,1)-path(idx-1,1);
    vy= path(idx,2)-path(idx-1,2);
end

targetDegree =atan(vy/vx)*180/pi;

end

function pos = predict_pos(pos, vel, T)

c = cos(pos(3)*pi/180);
s = sin(pos(3)*pi/180);
pos(1:2) = pos(1:2) + vel(1)*T*[c s];
pos(3) = pos(3) + vel(2)*T;

% DETERMINE DEGREE TO LIE BETWEEN -180~180
while pos(3) > 180
    pos(3) = pos(3) - 360;
end
while pos(3) < -180
    pos(3) = pos(3) + 360;
end

end