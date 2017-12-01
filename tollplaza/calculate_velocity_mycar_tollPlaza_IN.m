function mycar = calculate_velocity_mycar_tollPlaza_IN(mycar, sim, othercars, idm, laneChangePath, PLOT_MYCAR_DETECTING_AREA)


idx_observedcar = [];
idx_observing_mycar = [];
mycar.acceleration = [];

FLAG_OTHERCAR_INTENTION_EST = 1;

    % if entering the plaza
if mycar.flgPlaza == 0 && mycar.pos(1) > 100*10^3
    mycar.flgPlaza = 1;
    
    mycar.pathTranslated = laneChangePath{mycar.goallane, mycar.save.lane_idx};
end

if mycar.flgPlaza == 0 % before entering plaza
    idm.v0 = 15000;
    
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



% estimate crashing othercar and identify approaching othercar in front of mycar----------------------
[idx_observedcar, t_observedcar, pos_mycarEst, pos_observedcarEst, idx_observing_mycar, angle_observing_mycar, mycarpos_observing_mycar, t_observing_mycar] = is_carcrashed_orFollow_formycar_TTC_forwardrect(othercars, mycar, laneChangePath, FLAG_OTHERCAR_INTENTION_EST);


if PLOT_MYCAR_DETECTING_AREA
    
    [mycar.squareX, mycar.squareY] = make_detecting_rectangle(mycar, mycar.pos, mycar.vel, laneChangePath, mycar.detect_rect_forwardtime, mycar.detect_rect_sidewidth);
    
end


if ~isempty(idx_observedcar)
    
    nr_observedCar = length(idx_observedcar);
    idx_maxDecelerate = [];
    t_maxDecelerate = [];
    min_acceleration = 0;
    
    % iterate by number of estimated collision cars (to calculate deceleration by TTC)
    for i = 1:nr_observedCar
        
        rel_degree = abs(pos_mycarEst(i,3) - pos_observedcarEst(i,3));

        cur_acceleration = calculate_acceleration_IDM(mycar, othercars.car{idx_observedcar(i)}, pos_observedcarEst, idm, rel_degree);
        
        %fprintf(1, 'after [%d] seconds, mycar and [%d](%d, %d) collide at (%d, %d)\n', t, idx_observedcar, othercars.car{idx_observedcar(i)}.pos(1), othercars.car{idx_observedcar(i)}.pos(2), pos_mycarEst(1), pos_mycarEst(2));
        
        if i == 1
            min_acceleration = cur_acceleration;
        end
        
        if cur_acceleration <= min_acceleration
            mycar.acceleration = cur_acceleration;
            idx_maxDecelerate = idx_observedcar(i);
            t_maxDecelerate = t_observedcar(i);
        end
    end
    
    if mycar.acceleration < -2940
        fprintf(2, 'mycar([%d, %d]) decelerate([%d]) to car [%d] (observed time = [%d], reldegree = [%d])\n', mycar.pos(1), mycar.pos(2), mycar.acceleration, idx_maxDecelerate, t_maxDecelerate, rel_degree);
    else
        fprintf(1, 'mycar([%d, %d]) decelerate([%d]) to car [%d] (observed time = [%d], reldegree = [%d])\n', mycar.pos(1), mycar.pos(2), mycar.acceleration, idx_maxDecelerate, t_maxDecelerate, rel_degree);
    end
else
    A1 = mycar.vel(1)/idm.v0;
    mycar.acceleration = idm.a*(1 - A1^idm.delta);
end



if ~isempty(idx_observing_mycar)
    % 自車を前方検知予測した他車の数分だけ、角度から交錯か追従か判断し、あるべき減速度を求め、それより早いorあるべき減速度が0.3Gを下回る場合、自社は譲る（一定減速度で減速）
    nr_observing_mycar = length(idx_observing_mycar);
    idx_maxDecelerate = [];
    t_maxDecelerate = [];
    min_acceleration = 0;
    
    % iterate by number of estimated collision cars (to calculate deceleration by TTC)
    for i = 1:nr_observing_mycar

        rel_degree = angle_observing_mycar;
        
        %fprintf(1, 'after [%d] seconds, mycar and [%d](%d, %d) collide at (%d, %d)\n', t, idx_observedcar, othercars.car{idx_observedcar(i)}.pos(1), othercars.car{idx_observedcar(i)}.pos(2), pos_mycarEst(1), pos_mycarEst(2));
        
        mycar_posEst_i = mycarpos_observing_mycar(i,:);
        
        moderate_acceleration = calculate_acceleration_IDM(othercars.car{idx_observing_mycar(i)}, mycar, mycar_posEst_i, idm, rel_degree);
        
        
        if rel_degree >= 5
            fprintf(1, 'Car[%d]: moderate acceleration to mycar(reldegree = [%d] time = [%d]) is [%d]. Actual deceleration is [%d]\n', idx_observing_mycar(i), angle_observing_mycar(i), t_observing_mycar(i), moderate_acceleration, othercars.car{idx_observing_mycar(i)}.acceleration);
            % if (moderate_acceleration < -2940 || othercars.car{idx_observing_mycar(i)}.acceleration > moderate_acceleration) && mycar.acceleration > -2940
            if othercars.car{idx_observing_mycar(i)}.acceleration > moderate_acceleration || moderate_acceleration < -2940
                % mycar.acceleration = -2940;
                cur_mycar_acceleration_byrearcar = calculate_acceleration_IDM(mycar, othercars.car{idx_observing_mycar(i)}, mycarpos_observing_mycar, idm, rel_degree);
                
            end
        end
        
        if i == 1
            min_acceleration = cur_mycar_acceleration_byrearcar;
        elseif cur_mycar_acceleration_byrearcar < min_acceleration
            min_acceleration = cur_mycar_acceleration_byrearcar;
        end
        
    end
    
    if isempty(mycar.acceleration) || min_acceleration < mycar.acceleration
        mycar.acceleration = min_acceleration;
        fprintf(2, 'mycar([%d, %d]) decelerate([%d]) to car [%d] as it doesnot slow down\n', mycar.pos(1), mycar.pos(2), mycar.acceleration, idx_observing_mycar(i));
    end
end

mycar.vel(1) = mycar.vel(1) + mycar.acceleration*sim.T;


% control minimum velocity
if mycar.vel(1) < 0 && mycar.pos(1) < 320 * 10^3
    mycar.vel(1) = 0;
end

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