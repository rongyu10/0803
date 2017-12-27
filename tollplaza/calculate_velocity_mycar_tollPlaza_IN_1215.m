function mycar = calculate_velocity_mycar_tollPlaza_IN_1215(mycar, sim, othercars, idm, laneChangePath, MYCAR_AGGRESSIVE_MODE)

persistent idx_crossingcar;
persistent idx_crossingcar_pre;
mycar.acceleration = [];
persistent FLAG_FRONT_OR_BACK;
persistent acceleration_if_front;
persistent acceleration_if_back;
idx_precedingcar = [];

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
    
end




% 交錯予測がされる他車を検出する
if mycar.pos(1) > mycar.x_start_detecting
    idx_crossingcar_pre = idx_crossingcar;
    [idx_crossingcar, arr_t_mycar, arr_t_othercar, dist_section_mycar, rel_deg_crossingcar, sidepoint, invadepoint] = detect_crossing_car_1215(othercars, mycar);
end



% 交錯予測がされる他車がない場合、追従対象車を検出する
[mycar.squareX, mycar.squareY] = make_detecting_rectangle_lengthfix(mycar, mycar.pos, laneChangePath, mycar.detect_length, mycar.detect_sidewidth);
if isempty(idx_crossingcar)
        [idx_precedingcar, rel_deg_precedingcar] = detect_preceding_car_rectangle(othercars, mycar);
end



if ~isempty(idx_crossingcar) % 交錯対象車が検出された場合
    
    % 交錯予測がされる他車より2秒早く交錯点に到達するための加速度を計算（自車が他車より先行する）
    acceleration_if_front = 2*(dist_section_mycar - mycar.vel(1)*(arr_t_othercar - mycar.time_mergin_crossing)) / (arr_t_othercar - mycar.time_mergin_crossing)^2;
    
    % 交錯予測がされる他車より2秒遅く交錯点に到達するための加速度を計算（自車が他車に譲る）
    acceleration_if_back = 2*(dist_section_mycar - mycar.vel(1)*(arr_t_othercar + mycar.time_mergin_crossing)) / (arr_t_othercar + mycar.time_mergin_crossing)^2;
    
    % 前ステップで「先行or譲る」判断をしていない or 前ステップと対象他車のインデックスが異なる場合、「先行or譲る」判断をする
    if isempty(FLAG_FRONT_OR_BACK) || idx_crossingcar ~= idx_crossingcar_pre
        
        fprintf(2, 'acceleration_if_front = [%d], acceleration_if_back = [%d]\n', acceleration_if_front, acceleration_if_back);
        
        
        if arr_t_mycar - arr_t_othercar > mycar.maxtime_judge_front_or_back % 自車が他車より一定時間以上遅く交錯点に到達することが予測されたら「譲る」判断をする
            FLAG_FRONT_OR_BACK = 1;
            
        elseif arr_t_mycar - arr_t_othercar < -mycar.maxtime_judge_front_or_back % 自車が他車より一定時間以上早く交錯点に到達することが予測されたら「先行」判断をする
            FLAG_FRONT_OR_BACK = 0;
            
        else % 自車と他車の通過時刻差が一定時間以内の場合、「先行or譲る」の判定を行う
            switch MYCAR_AGGRESSIVE_MODE % 数字が大きいほどアグレッシブ（先行しやすい）判断をする
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
        end
        
        % 自車が先を行く判断をした時でも、他車を急減速させてしまうなら自車は一定減速度で減速
        if FLAG_FRONT_OR_BACK == 0
            dist_my2invadepoint = norm(invadepoint(1:2) - mycar.pos(1:2));
            time_invadepoint = (-mycar.vel(1)+sqrt(mycar.vel(1)^2 + 2*acceleration_if_front*dist_my2invadepoint)) / acceleration_if_front;
            other_futuresidepoint = update_pos([sidepoint othercars.car{idx_crossingcar}.pos(3)], othercars.car{idx_crossingcar}.vel, time_invadepoint);
            othercar_future.pos = other_futuresidepoint;
            othercar_future.vel = othercars.car{idx_crossingcar}.vel;
            mycar_future.pos = invadepoint;
            mycar_future.vel = mycar.vel;
            acceleration = calculate_acceleration_IDM(othercar_future, mycar_future, idm, rel_deg_crossingcar);
            if acceleration < -mycar.max_acceleration
                FLAG_FRONT_OR_BACK = 1;
                acceleration_if_back = -mycar.max_acceleration;
            end
        end
    end
    
    %「先行or譲る」判断結果により、自車の加速度を決定
    if FLAG_FRONT_OR_BACK == 0
        mycar.acceleration = acceleration_if_front;
    else
        mycar.acceleration = acceleration_if_back;
    end
    
elseif ~isempty(idx_precedingcar) % 追従対象車が検出された場合
    mycar.acceleration = calculate_acceleration_IDM(mycar, othercars.car{idx_precedingcar}, idm, rel_deg_precedingcar);
    
else % 交錯対象も追従対象も検出されなかった場合
    A1 = mycar.vel(1)/idm.v0;
    mycar.acceleration = idm.a*(1 - A1^idm.delta);
    FLAG_FRONT_OR_BACK = [];
    acceleration_if_front = [];
    acceleration_if_back = [];
end


% 自車の加減速度が最大値以上なら最大値に設定
if mycar.acceleration > mycar.max_acceleration
    mycar.acceleration = mycar.max_acceleration;
elseif mycar.acceleration < -mycar.max_acceleration
    mycar.acceleration = -mycar.max_acceleration;
end

% 自車速度を更新
mycar.vel(1) = mycar.vel(1) + mycar.acceleration*sim.T;

% 自車速度が負なら0に修正
if mycar.vel(1) < 0
    mycar.vel(1) = 0;
end

% コマンド出力-----------------------------------------------
if mycar.acceleration < -mycar.max_acceleration || mycar.acceleration > mycar.max_acceleration
    fprintf(2, 'mycar acceleration is [%d], velocity is [%d]\n', mycar.acceleration, mycar.vel(1));
else
    fprintf(1, 'mycar acceleration is [%d], velocity is [%d]\n', mycar.acceleration, mycar.vel(1));
end

if ~isempty(idx_crossingcar)
    if mycar.acceleration < -mycar.max_acceleration || mycar.acceleration > mycar.max_acceleration
        fprintf(2, '<Cross>mycar([%d, %d]) decelerate to car [%d] (arr_t_mycar=[%d], arr_t_othercar=[%d], dist_section_mycar=[%d], reldegree = [%d])\n', mycar.pos(1), mycar.pos(2), idx_crossingcar, arr_t_mycar, arr_t_othercar, dist_section_mycar, rel_deg_crossingcar);
    else
        fprintf(1, '<Cross>mycar([%d, %d]) decelerate to car [%d] (arr_t_mycar=[%d], arr_t_othercar=[%d], dist_section_mycar=[%d], reldegree = [%d])\n', mycar.pos(1), mycar.pos(2), idx_crossingcar, arr_t_mycar, arr_t_othercar, dist_section_mycar, rel_deg_crossingcar);
    end
end

if ~isempty(idx_precedingcar)
    if mycar.acceleration < -mycar.max_acceleration || mycar.acceleration > mycar.max_acceleration
        fprintf(2, '<Precede>mycar([%d, %d]) decelerate to car [%d] by IDM\n', mycar.pos(1), mycar.pos(2), idx_precedingcar);
    else
        fprintf(1, '<Precede>mycar([%d, %d]) decelerate to car [%d] by IDM\n', mycar.pos(1), mycar.pos(2), idx_precedingcar);
    end
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