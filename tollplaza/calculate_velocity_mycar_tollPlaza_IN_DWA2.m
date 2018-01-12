function mycar = calculate_velocity_mycar_tollPlaza_IN_DWA2(mycar, sim, othercars, idm, laneChangePath)

persistent idx_crossingcar;
mycar.acceleration = [];
idx_precedingcar = [];

% setting of DWA
RangeDWA=[0 30000 2940*sim.T 10]; %[最低速度(mm/s)　最高速度(mm/s)　速度レンジ(m/s)　速度解像度(分割数)]
ParamDWA=[1.0 1.0 1.0]; %[現在の速度を維持する項　前方車との予測通過時間差の項　後方車を急減速させない項]　


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
    [idx_crossingcar, arr_t_othercar, dist_section_mycar, rel_deg_crossingcar, sidepoint, mycar.invadepoint] = detect_crossing_car_1215(othercars, mycar);
    
end



% 交錯予測がされる他車がない場合、追従対象車を検出する
[mycar.squareX, mycar.squareY] = make_detecting_rectangle_lengthfix(mycar, mycar.pos, laneChangePath, mycar.detect_length, mycar.detect_sidewidth);
if isempty(idx_crossingcar)
        [idx_precedingcar, rel_deg_precedingcar] = detect_preceding_car_rectangle(othercars, mycar, sim);
end


if ~isempty(idx_crossingcar) % 交錯対象車が検出された場合
    
    evalDWA = zeros(RangeDWA(4) + 1, 5); % 評価値を格納する行列
    
    nr_cars = length(idx_crossingcar);
    
    for i = 1:RangeDWA(4) + 1
        
        evalDWA(i,1) = mycar.vel(1) - RangeDWA(3) + 2 * RangeDWA(3)/ RangeDWA(4) * (i - 1);
        
        % 各サンプリング速度から加速度を計算（以下、サンプリング加速度と呼ぶ）
        est_acceleration = (evalDWA(i,1) - mycar.vel(1)) / sim.T;
        
        % 各サンプリング速度が最小（大）速度を越えていたら評価しない
        if evalDWA(i,1) < RangeDWA(1) || evalDWA(i,1) > RangeDWA(2)
            continue
        end
        
        flg_notEval = 0;
        iFrontcar = [];
        iRearcar = [];
        est_arr_t_mycar = zeros(nr_cars);
        for iCars = 1:nr_cars
            % 各サンプリング加速度で減速したとき、他車との交点に到達するまで速度がマイナスになる場合、評価しない（下のルート内がマイナスになるとき）
            if mycar.vel(1)^2 + 2*est_acceleration*dist_section_mycar(iCars) < 0
                flg_notEval = 1;
                continue
            end
            
            if est_acceleration ~= 0
                est_arr_t_mycar(iCars) = (-mycar.vel(1)+sqrt(mycar.vel(1)^2 + 2*est_acceleration*dist_section_mycar(iCars))) / est_acceleration;
            else
                est_arr_t_mycar(iCars) = dist_section_mycar(iCars) / evalDWA(i,1);
            end
            
            % もっとも近い前方車、後方車を求める
            if est_arr_t_mycar(iCars) - arr_t_othercar(iCars) > 0
                if isempty(iFrontcar)
                    iFrontcar = iCars;
                else
                    if est_arr_t_mycar(iCars) - arr_t_othercar(iCars) < est_arr_t_mycar(iFrontcar) - arr_t_othercar(iFrontcar)
                        iFrontcar = iCars;
                    end
                end
            else
                if isempty(iRearcar)
                    iRearcar = iCars;
                else
                    if est_arr_t_mycar(iCars) - arr_t_othercar(iCars) < est_arr_t_mycar(iRearcar) - arr_t_othercar(iRearcar)
                        iRearcar = iCars;
                    end
                end
            end
        end
        if flg_notEval == 1
            continue
        end
        
        % 現在速度を維持する評価値を計算
        evalDWA(i,3) = ParamDWA(1) * (1 - abs(evalDWA(i,1) - mycar.vel(1))/RangeDWA(3));
        
        % 前方車に衝突しない評価値（前方車との交点通過時間差）を計算
        if ~isempty(iFrontcar)
            if est_arr_t_mycar(iFrontcar) - arr_t_othercar(iFrontcar) > mycar.time_mergin_crossing
                evalDWA(i,4) = 0;
            else
                evalDWA(i,4) = - ParamDWA(2) * (mycar.time_mergin_crossing - (est_arr_t_mycar(iFrontcar) - arr_t_othercar(iFrontcar))) / mycar.time_mergin_crossing;
            end
        end
        
        % 後方車を減速させない評価値（自車の交点通過予測時における後方車との相対位置からIDMで評価）を計算
        if ~isempty(iRearcar)
            other_futuresidepoint = update_pos([sidepoint(iRearcar,:) othercars.car{idx_crossingcar(iRearcar)}.pos(3)], othercars.car{idx_crossingcar(iRearcar)}.vel, est_arr_t_mycar(iRearcar));
            othercar_future.pos = other_futuresidepoint;
            othercar_future.vel = othercars.car{idx_crossingcar(iRearcar)}.vel;
            mycar_future.pos = mycar.invadepoint(iRearcar,:);
            mycar_future.vel(1) = mycar.vel(1) + est_acceleration*est_arr_t_mycar(iRearcar);
            mycar_future.vel(2) = mycar.vel(2);
            est_other_accele = calculate_acceleration_IDM(othercar_future, mycar_future, idm, rel_deg_crossingcar(iRearcar));
            
            if est_other_accele > 0
                evalDWA(i,5) = 0;
            else
                evalDWA(i,5) = ParamDWA(3) * est_other_accele/mycar.max_acceleration;
            end
        end
        
        
        % 評価値の合計を計算
        evalDWA(i,2) = evalDWA(i,3) + evalDWA(i,4) + evalDWA(i,5);
        
    end
    fprintf(1, '観測他車台数：[%d]\n', nr_cars);
    fprintf(1, '現在の速度を維持する項：(-5:%.3f, -4:%.3f, -3:%.3f, -2:%.3f, -1:%.3f, 0:%.3f, 1:%.3f, 2:%.3f, 3:%.3f, 4:%.3f, 5:%.3f) \n', evalDWA(1,3), evalDWA(2,3), evalDWA(3,3), evalDWA(4,3), evalDWA(5,3), evalDWA(6,3), evalDWA(7,3), evalDWA(8,3), evalDWA(9,3), evalDWA(10,3), evalDWA(11,3));
    fprintf(1, '前方車とのTHWをとる項：(-5:%.3f, -4:%.3f, -3:%.3f, -2:%.3f, -1:%.3f, 0:%.3f, 1:%.3f, 2:%.3f, 3:%.3f, 4:%.3f, 5:%.3f) \n', evalDWA(1,4), evalDWA(2,4), evalDWA(3,4), evalDWA(4,4), evalDWA(5,4), evalDWA(6,4), evalDWA(7,4), evalDWA(8,4), evalDWA(9,4), evalDWA(10,4), evalDWA(11,4));
    fprintf(1, '後方車を急減速させない項：(-5:%.3f, -4:%.3f, -3:%.3f, -2:%.3f, -1:%.3f, 0:%.3f, 1:%.3f, 2:%.3f, 3:%.3f, 4:%.3f, 5:%.3f) \n', evalDWA(1,5), evalDWA(2,5), evalDWA(3,5), evalDWA(4,5), evalDWA(5,5), evalDWA(6,5), evalDWA(7,5), evalDWA(8,5), evalDWA(9,5), evalDWA(10,5), evalDWA(11,5));
    fprintf(1, '評価値合計：(-5:%.3f, -4:%.3f, -3:%.3f, -2:%.3f, -1:%.3f, 0:%.3f, 1:%.3f, 2:%.3f, 3:%.3f, 4:%.3f, 5:%.3f) \n', evalDWA(1,2), evalDWA(2,2), evalDWA(3,2), evalDWA(4,2), evalDWA(5,2), evalDWA(6,2), evalDWA(7,2), evalDWA(8,2), evalDWA(9,2), evalDWA(10,2), evalDWA(11,2));
    
    [~, idx] = max(evalDWA(:,2));
    mycar.acceleration = (evalDWA(idx,1) - mycar.vel(1)) / sim.T;
    
elseif ~isempty(idx_precedingcar) % 追従対象車が検出された場合
    mycar.acceleration = calculate_acceleration_IDM(mycar, othercars.car{idx_precedingcar}, idm, rel_deg_precedingcar);
    
else % 交錯対象も追従対象も検出されなかった場合
    A1 = mycar.vel(1)/idm.v0;
    mycar.acceleration = idm.a*(1 - A1^idm.delta);
    FLAG_FRONT_OR_BACK = [];
    acceleration_if_front = [];
    acceleration_if_back = [];
end

% 自車速度を更新
mycar.vel(1) = mycar.vel(1) + mycar.acceleration*sim.T;

% 自車速度が負なら0に修正
if mycar.vel(1) < 0
    mycar.vel(1) = 0;
end

% コマンド出力-----------------------------------------------
if mycar.acceleration <= -mycar.max_acceleration || mycar.acceleration >= mycar.max_acceleration
    fprintf(2, 'mycar[%6.0f, %4.0f] acceleration is [%4.0f], velocity is [%5.0f]\n\n', mycar.pos(1), mycar.pos(2), mycar.acceleration, mycar.vel(1));
else
    fprintf(1, 'mycar[%6.0f, %4.0f] acceleration is [%4.0f], velocity is [%5.0f]\n\n', mycar.pos(1), mycar.pos(2), mycar.acceleration, mycar.vel(1));
end

% if ~isempty(idx_crossingcar)
%     if mycar.acceleration <= -mycar.max_acceleration || mycar.acceleration >= mycar.max_acceleration
%         fprintf(2, '<Cross>mycar[%6.0f, %4.0f] with car[%d]%d (-5:%.3f, -4:%.3f, -3:%.3f, -2:%.3f, -1:%.3f, 0:%.3f, 1:%.3f, 2:%.3f, 3:%.3f, 4:%.3f, 5:%.3f) \n', mycar.pos(1), mycar.pos(2), idx_crossingcar(1), nr_cars, sum_evalDWA(1), sum_evalDWA(2), sum_evalDWA(3), sum_evalDWA(4), sum_evalDWA(5), sum_evalDWA(6), sum_evalDWA(7), sum_evalDWA(8), sum_evalDWA(9), sum_evalDWA(10), sum_evalDWA(11));
%     else
%         fprintf(1, '<Cross>mycar[%6.0f, %4.0f] with car[%d]%d (-5:%.3f, -4:%.3f, -3:%.3f, -2:%.3f, -1:%.3f, 0:%.3f, 1:%.3f, 2:%.3f, 3:%.3f, 4:%.3f, 5:%.3f) \n', mycar.pos(1), mycar.pos(2), idx_crossingcar(1), nr_cars, sum_evalDWA(1), sum_evalDWA(2), sum_evalDWA(3), sum_evalDWA(4), sum_evalDWA(5), sum_evalDWA(6), sum_evalDWA(7), sum_evalDWA(8), sum_evalDWA(9), sum_evalDWA(10), sum_evalDWA(11));
%     end
% end

if ~isempty(idx_precedingcar)
    if mycar.acceleration < -mycar.max_acceleration || mycar.acceleration > mycar.max_acceleration
        fprintf(2, '<Follow>mycar([%6.0f, %4.0f]) decelerate to car [%d] by IDM\n', mycar.pos(1), mycar.pos(2), idx_precedingcar);
    else
        fprintf(1, '<Follow>mycar([%6.0f, %4.0f]) decelerate to car [%d] by IDM\n', mycar.pos(1), mycar.pos(2), idx_precedingcar);
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