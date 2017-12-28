function mycar = calculate_velocity_mycar_tollPlaza_IN_DWA(mycar, sim, othercars, idm, laneChangePath)

persistent idx_crossingcar;
mycar.acceleration = [];
idx_precedingcar = [];

% setting of DWA
RangeDWA=[0 20000 2940*sim.T 10]; %[最低速度(mm/s)　最高速度(mm/s)　速度レンジ(m/s)　速度解像度(分割数)]
ParamDWA=[1.0 3.0 1.0]; %[現在の速度を維持する項　他車との予測通過時間差の項　他車を急減速させない項]　


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
    [idx_crossingcar, arr_t_mycar, arr_t_othercar, dist_section_mycar, rel_deg_crossingcar, sidepoint, invadepoint] = detect_crossing_car_1215(othercars, mycar);
end



% 交錯予測がされる他車がない場合、追従対象車を検出する
[mycar.squareX, mycar.squareY] = make_detecting_rectangle_lengthfix(mycar, mycar.pos, laneChangePath, mycar.detect_length, mycar.detect_sidewidth);
if isempty(idx_crossingcar)
        [idx_precedingcar, rel_deg_precedingcar] = detect_preceding_car_rectangle(othercars, mycar, sim);
end


if ~isempty(idx_crossingcar) % 交錯対象車が検出された場合
    
    evalDWA = zeros(RangeDWA(4) + 1, 5);
    for i = 1:RangeDWA(4) + 1
        % 現在速度を維持する評価値を計算
        evalDWA(i,1) = mycar.vel(1) - RangeDWA(3) + 2 * RangeDWA(3)/ RangeDWA(4) * (i - 1);
        if evalDWA(i,1) < RangeDWA(1) || evalDWA(i,1) > RangeDWA(2)
           continue
        else
           evalDWA(i,3) = ParamDWA(1) * (1 - abs(evalDWA(i,1) - mycar.vel(1))/RangeDWA(3));
        end
        
        % 他車に衝突しない評価値（他車との交点通過時間差）を計算
        est_acceleration = (evalDWA(i,1) - mycar.vel(1)) / sim.T;
        if est_acceleration ~= 0
            est_arr_t_mycar = (-mycar.vel(1)+sqrt(mycar.vel(1)^2 + 2*est_acceleration*dist_section_mycar)) / est_acceleration;
        else
            est_arr_t_mycar = dist_section_mycar / evalDWA(i,1);
        end
        if abs(est_arr_t_mycar - arr_t_othercar) > mycar.time_mergin_crossing
            evalDWA(i,4) = ParamDWA(2);
        else
            evalDWA(i,4) = ParamDWA(2) * abs(est_arr_t_mycar - arr_t_othercar) / mycar.time_mergin_crossing;
        end
        
        % 他者を減速させない評価値（自車の交点通過予測時における他者との相対位置からIDMで評価）を計算
        if est_arr_t_mycar > arr_t_othercar
            evalDWA(i,5) = 0;
        else
            
            other_futuresidepoint = update_pos([sidepoint othercars.car{idx_crossingcar}.pos(3)], othercars.car{idx_crossingcar}.vel, est_arr_t_mycar);
            othercar_future.pos = other_futuresidepoint;
            othercar_future.vel = othercars.car{idx_crossingcar}.vel;
            mycar_future.pos = invadepoint;
            mycar_future.vel(1) = mycar.vel(1) + est_acceleration*est_arr_t_mycar;
            mycar_future.vel(2) = mycar.vel(2);
            est_other_accele = calculate_acceleration_IDM(othercar_future, mycar_future, idm, rel_deg_crossingcar);
            
            if est_other_accele > 0
                evalDWA(i,5) = 0;
            elseif est_other_accele > -mycar.max_acceleration
                evalDWA(i,5) = ParamDWA(3) * est_other_accele/mycar.max_acceleration;
            else
                evalDWA(i,5) = -ParamDWA(3);
            end
            
        end
        
        
        % 評価値の合計を計算
        evalDWA(i,2) = evalDWA(i,3) + evalDWA(i,4) + evalDWA(i,5);
    end
    
    
    
    [~, idx] = max(evalDWA(:,2));
    mycar.acceleration = (evalDWA(idx, 1) - mycar.vel(1)) / sim.T;
    
    
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
% if mycar.acceleration > mycar.max_acceleration
%     mycar.acceleration = mycar.max_acceleration;
% elseif mycar.acceleration < -mycar.max_acceleration
%     mycar.acceleration = -mycar.max_acceleration;
% end

% 自車速度を更新
mycar.vel(1) = mycar.vel(1) + mycar.acceleration*sim.T;

% 自車速度が負なら0に修正
if mycar.vel(1) < 0
    mycar.vel(1) = 0;
end

% コマンド出力-----------------------------------------------
if mycar.acceleration <= -mycar.max_acceleration || mycar.acceleration >= mycar.max_acceleration
    fprintf(2, 'mycar acceleration is [%d], velocity is [%d]\n', mycar.acceleration, mycar.vel(1));
else
    fprintf(1, 'mycar acceleration is [%d], velocity is [%d]\n', mycar.acceleration, mycar.vel(1));
end

if ~isempty(idx_crossingcar)
    if mycar.acceleration <= -mycar.max_acceleration || mycar.acceleration >= mycar.max_acceleration
        fprintf(2, '<Cross>mycar([%d, %d]) decelerate to car [%d] (arr_t_mycar=[%d], arr_t_othercar=[%d], dist_section_mycar=[%d], reldegree = [%d])\n', mycar.pos(1), mycar.pos(2), idx_crossingcar, arr_t_mycar, arr_t_othercar, dist_section_mycar, rel_deg_crossingcar);
    else
        fprintf(1, '<Cross>mycar([%d, %d]) decelerate to car [%d] (arr_t_mycar=[%d], arr_t_othercar=[%d], dist_section_mycar=[%d], reldegree = [%d])\n', mycar.pos(1), mycar.pos(2), idx_crossingcar, arr_t_mycar, arr_t_othercar, dist_section_mycar, rel_deg_crossingcar);
    end
end

if ~isempty(idx_precedingcar)
    if mycar.acceleration < -mycar.max_acceleration || mycar.acceleration > mycar.max_acceleration
        fprintf(2, '<Follow>mycar([%d, %d]) decelerate to car [%d] by IDM\n', mycar.pos(1), mycar.pos(2), idx_precedingcar);
    else
        fprintf(1, '<Follow>mycar([%d, %d]) decelerate to car [%d] by IDM\n', mycar.pos(1), mycar.pos(2), idx_precedingcar);
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