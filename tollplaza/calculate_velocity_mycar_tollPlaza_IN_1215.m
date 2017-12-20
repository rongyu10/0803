function mycar = calculate_velocity_mycar_tollPlaza_IN_1215(mycar, sim, othercars, idm, laneChangePath, MYCAR_AGGRESSIVE_MODE)

persistent idx_crossingcar;
persistent idx_crossingcar_pre;
mycar.acceleration = [];
persistent FLAG_FRONT_OR_BACK;
persistent acceleration_if_front;
persistent acceleration_if_back;

% if entering the plaza
if mycar.flgPlaza == 0 && mycar.pos(1) > 100*10^3
    mycar.flgPlaza = 1;
    
end

if mycar.flgPlaza == 0 % before entering plaza
    idm.v0 = 17500; % 63km/h
    mycar.pathTranslated = laneChangePath{mycar.goallane, mycar.save.lane_idx};
    
elseif mycar.flgPlaza == 1 % after entering plaza
    pos = predict_pos(mycar.pos, mycar.vel, sim.T);
    mycar.targetDegree = get_tatgetTheta(pos,mycar.pathTranslated);
    mycar.vel(2) = (mycar.targetDegree - mycar.pos(3))/sim.T;
    
    % 目標速度をゲートからの距離に応じて27km/h〜63km/hに設定
    idm.v0 = 7500 + 10000*(320*10^3 - mycar.pos(1))/(220*10^3);
    
%     if mycar.pos(1) < 187.5*10^3
%         idm.v0 = 15000;
% 
%     elseif mycar.pos(1) < 275*10^3
%         idm.v0 = 12500;
%         
%     elseif mycar.pos(1) < 320*10^3
%         idm.v0 = 10000;
%     end
    
end

% [mycar.squareX, mycar.squareY] = make_detecting_rectangle_lengthfix(mycar, mycar.pos, laneChangePath, mycar.detect_rect_length, mycar.detect_rect_sidewidth);


% estimate crashing othercar and identify approaching othercar in front of mycar----------------------
if mycar.pos(1) > mycar.x_start_detecting
%     if isempty(idx_crossingcar)
%         [idx_precedingcar, rel_deg_precedingcar] = detect_preceding_car_1123(othercars, mycar);
%         FLAG_FRONT_OR_BACK = [];
%     end
    idx_crossingcar_pre = idx_crossingcar;
    [idx_crossingcar, arr_t_mycar, arr_t_othercar, dist_section_mycar, rel_deg_crossingcar, sidepoint, invadepoint] = detect_crossing_car_1215(othercars, mycar);
end

if ~isempty(idx_crossingcar)
    % 両車の交点通過予測時間差が設定時間以上 or 既に「先行or譲る」判断が済んでいる場合、自由走行
    if abs(arr_t_othercar - arr_t_mycar) > mycar.max_time_dif_intersection && isempty(FLAG_FRONT_OR_BACK)
        A1 = mycar.vel(1)/idm.v0;
        mycar.acceleration = idm.a*(1 - A1^idm.delta);
    else
        
        if isempty(FLAG_FRONT_OR_BACK) || idx_crossingcar ~= idx_crossingcar_pre% if decision(front or back) is not determined
            
            acceleration_if_front = 2*(dist_section_mycar - mycar.vel(1)*(arr_t_othercar - mycar.time_mergin_crossing)) / (arr_t_othercar - mycar.time_mergin_crossing)^2;
            
            acceleration_if_back = 2*(dist_section_mycar - mycar.vel(1)*(arr_t_othercar + mycar.time_mergin_crossing)) / (arr_t_othercar + mycar.time_mergin_crossing)^2;
            
            
            switch MYCAR_AGGRESSIVE_MODE
                case 0
                    if acceleration_if_back > -mycar.max_acceleration
                        FLAG_FRONT_OR_BACK = 1;
                    elseif acceleration_if_front < mycar.max_acceleration
                        FLAG_FRONT_OR_BACK = 0;
                    else
                        if abs(acceleration_if_front) < abs(acceleration_if_back)
                            FLAG_FRONT_OR_BACK = 0;
                        else
                            FLAG_FRONT_OR_BACK = 1;
                        end
                    end
                case 1
                    if abs(acceleration_if_front) < abs(acceleration_if_back)
                        FLAG_FRONT_OR_BACK = 0;
                    else
                        FLAG_FRONT_OR_BACK = 1;
                    end
                case 2
                    if acceleration_if_front < mycar.max_acceleration
                        FLAG_FRONT_OR_BACK = 0;
                    elseif acceleration_if_back > -mycar.max_acceleration
                        FLAG_FRONT_OR_BACK = 1;
                    else
                        if abs(acceleration_if_front) < abs(acceleration_if_back)
                            FLAG_FRONT_OR_BACK = 0;
                        else
                            FLAG_FRONT_OR_BACK = 1;
                        end
                    end 
            end
            
            if FLAG_FRONT_OR_BACK == 0
                if acceleration_if_front < 0 % 設定時間に他車の前を通過するために減速してしまう場合、自由走行
                    A1 = mycar.vel(1)/idm.v0;
                    mycar.acceleration = idm.a*(1 - A1^idm.delta);
                    
                    FLAG_FRONT_OR_BACK = [];
                    acceleration_if_front = [];
                    acceleration_if_back = [];
                else
                    mycar.acceleration = acceleration_if_front;
                    fprintf(2, 'acceleration_if_front = [%d], acceleration_if_back = [%d]\n', acceleration_if_front, acceleration_if_back);
                end
            elseif FLAG_FRONT_OR_BACK == 1
                if acceleration_if_back > 0 % 設定時間に他車の後を通過するために加速してしまう場合、自由走行
                    A1 = mycar.vel(1)/idm.v0;
                    mycar.acceleration = idm.a*(1 - A1^idm.delta);
                    
                    FLAG_FRONT_OR_BACK = [];
                    acceleration_if_front = [];
                    acceleration_if_back = [];
                else
                    mycar.acceleration = acceleration_if_back;
                    fprintf(2, 'acceleration_if_front = [%d], acceleration_if_back = [%d]\n', acceleration_if_front, acceleration_if_back);
                end
            end
            
            
%             if FLAG_FRONT_OR_BACK == 0
%                 dist_my2invadepoint = norm(invadepoint(1:2) - mycar.pos(1:2));
%                 time_invadepoint = (-mycar.vel(1)+sqrt(mycar.vel(1)^2 + 2*mycar.acceleration*dist_my2invadepoint)) / mycar.acceleration;
%                 other_futuresidepoint = update_pos([sidepoint othercars.car{idx_crossingcar}.pos(3)], othercars.car{idx_crossingcar}.vel, time_invadepoint);
%                 othercar_future.pos = other_futuresidepoint;
%                 othercar_future.vel = othercars.car{idx_crossingcar}.vel;
%                 mycar_future.pos = invadepoint;
%                 mycar_future.vel = mycar.vel;
%                 acceleration = calculate_acceleration_IDM(othercar_future, mycar_future, idm, rel_deg_crossingcar);
%                 if acceleration < -mycar.max_acceleration
%                     FLAG_FRONT_OR_BACK = 1;
%                     mycar.acceleration = acceleration_if_back;
%                 end
%             end
            
        else
            if FLAG_FRONT_OR_BACK == 0
                mycar.acceleration = acceleration_if_front;
            else
                mycar.acceleration = acceleration_if_back;
            end
        end
        
    end
else
    A1 = mycar.vel(1)/idm.v0;
    mycar.acceleration = idm.a*(1 - A1^idm.delta);
    FLAG_FRONT_OR_BACK = [];
    acceleration_if_front = [];
    acceleration_if_back = [];
end


mycar.vel(1) = mycar.vel(1) + mycar.acceleration*sim.T;


% --regulate acceleration and velocity ---------------------
if mycar.acceleration > mycar.max_acceleration
    mycar.acceleration = mycar.max_acceleration;
elseif mycar.acceleration < -mycar.max_acceleration
    mycar.acceleration = -mycar.max_acceleration;
end

if mycar.acceleration < -mycar.max_acceleration || mycar.acceleration > mycar.max_acceleration
    fprintf(2, 'mycar acceleration is [%d], velocity is [%d]\n', mycar.acceleration, mycar.vel(1));
else
    fprintf(1, 'mycar acceleration is [%d], velocity is [%d]\n', mycar.acceleration, mycar.vel(1));
end

if ~isempty(idx_crossingcar)
    if mycar.acceleration < -mycar.max_acceleration || mycar.acceleration > mycar.max_acceleration
        fprintf(2, 'mycar([%d, %d]) decelerate to car [%d] (arr_t_mycar=[%d], arr_t_othercar=[%d], dist_section_mycar=[%d], reldegree = [%d])\n', mycar.pos(1), mycar.pos(2), idx_crossingcar, arr_t_mycar, arr_t_othercar, dist_section_mycar, rel_deg_crossingcar);
    else
        fprintf(1, 'mycar([%d, %d]) decelerate to car [%d] (arr_t_mycar=[%d], arr_t_othercar=[%d], dist_section_mycar=[%d], reldegree = [%d])\n', mycar.pos(1), mycar.pos(2), idx_crossingcar, arr_t_mycar, arr_t_othercar, dist_section_mycar, rel_deg_crossingcar);
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