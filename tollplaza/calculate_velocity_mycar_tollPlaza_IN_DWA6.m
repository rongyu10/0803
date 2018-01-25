function mycar = calculate_velocity_mycar_tollPlaza_IN_DWA6(mycar, sim, othercars, idm, laneChangePath, RangeDWA, ParamDWA)

persistent idx_crossingcar;
mycar.acceleration = [];
idx_precedingcar = [];




% if entering the plaza
if mycar.flgPlaza == 0 && mycar.pos(1) > 100*10^3
    mycar.flgPlaza = 1;
    
end

if mycar.flgPlaza == 0 % before entering plaza
    % idm.v0 = 17500; % 63km/h
    idm.v0 = 17500;
    mycar.pathTranslated = laneChangePath{mycar.goallane, mycar.save.lane_idx};
    
elseif mycar.flgPlaza == 1 % after entering plaza
    pos = predict_pos(mycar.pos, mycar.vel, sim.T);
    mycar.targetDegree = get_tatgetTheta(pos,mycar.pathTranslated);
    mycar.vel(2) = (mycar.targetDegree - mycar.pos(3))/sim.T;
    
    % 目標速度をゲートからの距離に応じて27km/h〜63km/hに設定
    idm.v0 = 7500 + 10000*(320*10^3 - mycar.pos(1))/(220*10^3);
    %idm.v0 = 5000;
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
    
    evalDWA = zeros(RangeDWA(4) + 1, 10); % 評価値を格納する行列
    
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
        iRearcar = [];
        iNearcar = [];
        est_arr_t_mycar = zeros(nr_cars);
        for iCars = 1:nr_cars
            % 各サンプリング加速度で減速したとき、他車との交点に到達するまで速度が0以下になる場合（下のルート内がマイナスになる場合）
            if mycar.vel(1)^2 + 2*est_acceleration*dist_section_mycar(iCars) <= 0
                continue
            end
            
            if est_acceleration ~= 0
                est_arr_t_mycar(iCars) = (-mycar.vel(1)+sqrt(mycar.vel(1)^2 + 2*est_acceleration*dist_section_mycar(iCars))) / est_acceleration;
            else
                est_arr_t_mycar(iCars) = dist_section_mycar(iCars) / evalDWA(i,1);
            end
            
            % 最も近い他車、最も近い前方車、最も近い後方車を求める
            if est_arr_t_mycar(iCars) - arr_t_othercar(iCars) < 0
                if isempty(iRearcar)
                    iRearcar = iCars;
                else
                    if est_arr_t_mycar(iCars) - arr_t_othercar(iCars) > est_arr_t_mycar(iRearcar) - arr_t_othercar(iRearcar)
                        iRearcar = iCars;
                    end
                end
            end
            
            if isempty(iNearcar)
                iNearcar = iCars;
            else
                if abs(est_arr_t_mycar(iCars) - arr_t_othercar(iCars)) < abs(est_arr_t_mycar(iNearcar) - arr_t_othercar(iNearcar))
                    iNearcar = iCars;
                end
            end
            
            
            
            % 自車の交点通過予測時における後方車の予測減速度を計算し、急減速なら評価しない
            if ~isempty(iRearcar)
                
                other_futuresidepoint = update_pos([sidepoint(iRearcar,:) othercars.car{idx_crossingcar(iRearcar)}.pos(3)], othercars.car{idx_crossingcar(iRearcar)}.vel, est_arr_t_mycar(iRearcar));
                othercar_future.pos = other_futuresidepoint;
                othercar_future.vel = othercars.car{idx_crossingcar(iRearcar)}.vel;
                mycar_future.pos = mycar.invadepoint(iRearcar,:);
                mycar_future.vel(1) = mycar.vel(1) + est_acceleration*est_arr_t_mycar(iRearcar);
                mycar_future.vel(2) = mycar.vel(2);
                est_other_accele = calculate_acceleration_IDM(othercar_future, mycar_future, idm, rel_deg_crossingcar(iRearcar));
                
                if est_other_accele < - mycar.let_othercar_decele
                    flg_notEval = 1;
                    continue
                end
            end
            
        end
        
        if ~isempty(iRearcar)
            evalDWA(i,9) = idx_crossingcar(iRearcar);
            evalDWA(i,10) = arr_t_othercar(iRearcar) - est_arr_t_mycar(iRearcar);
        end
        
        if ~isempty(iNearcar)
            if abs(est_arr_t_mycar(iNearcar) - arr_t_othercar(iNearcar)) * othercars.car{idx_crossingcar(iNearcar)}.vel(1) < 5*10^3
                continue
            end
        end
        
        if flg_notEval == 1
            continue
        end
        
        % 現在速度を維持する評価値を計算
        % evalDWA(i,3) = ParamDWA(1) * (1 - abs(evalDWA(i,1) - mycar.vel(1))/RangeDWA(3));
        evalDWA(i,3) = ParamDWA(1) * exp(-4*(((evalDWA(i,1) - mycar.vel(1))/RangeDWA(3))^2));
        
        % 最も近い他車に衝突しない評価値（交点通過時間差）を計算
        if ~isempty(iNearcar)
            % evalDWA(i,4) = ParamDWA(2) * (1 - exp(-4*(((est_arr_t_mycar(iNearcar) - arr_t_othercar(iNearcar))/mycar.time_margin_crossing)^2)));
            if abs(est_arr_t_mycar(iNearcar) - arr_t_othercar(iNearcar)) > mycar.time_margin_crossing
                evalDWA(i,8) = ParamDWA(2);
            else
                evalDWA(i,8) = ParamDWA(2) * (1 - exp(-4*(((est_arr_t_mycar(iNearcar) - arr_t_othercar(iNearcar))/mycar.time_margin_crossing)^2)));
            end
        else
            evalDWA(i,8) = ParamDWA(2);
        end
        
        if ~isempty(iNearcar)
            evalDWA(i,6) = idx_crossingcar(iNearcar);
            evalDWA(i,7) = arr_t_othercar(iNearcar) - est_arr_t_mycar(iNearcar);
        end
    end
    
    max_eval = max(evalDWA(:,8));
    min_eval = min(evalDWA(:,8));
    
    if max_eval - min_eval ~= 0
        for i = 1:RangeDWA(4) + 1
            evalDWA(i,4) = ParamDWA(2) * (evalDWA(i,8) - min_eval)/(max_eval - min_eval);
        end
    else
        evalDWA(:,4) = ParamDWA(2);
    end
    
    for i = 1:RangeDWA(4) + 1
        % 評価値の合計を計算
        evalDWA(i,2) = evalDWA(i,3) + evalDWA(i,4) + evalDWA(i,5);
    end
    
    fprintf(1, '観測他車：');
    for i=1:nr_cars
        fprintf(1, '[%d]', idx_crossingcar(i));
    end
    fprintf(1, '\n');
    
    fprintf(1, '現在速度維持項：(');
    for i=1:RangeDWA(4)+1
        fprintf(1, '[%d:% .3f] ', i, evalDWA(i,3));
    end
    fprintf(1, '\n');
    
    fprintf(1, '最も近い他車id：(');
    for i=1:RangeDWA(4)+1
        fprintf(1, '[%d:%6d] ', i, evalDWA(i,6));
    end
    fprintf(1, '\n');
    
    fprintf(1, '他車通過時間差：(');
    for i=1:RangeDWA(4)+1
        fprintf(1, '[%d:% .3f] ', i, evalDWA(i,7));
    end
    fprintf(1, '\n');
    
    fprintf(1, '正規化前距離項：(');
    for i=1:RangeDWA(4)+1
        fprintf(1, '[%d:% .3f] ', i, evalDWA(i,8));
    end
    fprintf(1, '\n');
    
    fprintf(1, '他車との距離項：(');
    for i=1:RangeDWA(4)+1
        fprintf(1, '[%d:% .3f] ', i, evalDWA(i,4));
    end
    fprintf(1, '\n');
    
    fprintf(1, '最も近い後方車：(');
    for i=1:RangeDWA(4)+1
        fprintf(1, '[%d:%6d] ', i, evalDWA(i,9));
    end
    fprintf(1, '\n');
    
    fprintf(1, '後車通過時間差：(');
    for i=1:RangeDWA(4)+1
        fprintf(1, '[%d:% .3f] ', i, evalDWA(i,10));
    end
    fprintf(1, '\n');
    
    fprintf(1, '上記３つの合計：(');
    for i=1:RangeDWA(4)+1
        fprintf(1, '[%d:% .3f] ', i, evalDWA(i,2));
    end
    fprintf(1, '\n');
    
    
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