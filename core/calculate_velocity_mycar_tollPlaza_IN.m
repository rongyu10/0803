function mycar = calculate_velocity_mycar_tollPlaza_IN(mycar, sim, othercars, idm, laneChangePath, PLOT_MYCAR_DETECTING_AREA)


% PARAMETER OF INTELLIGENT DRIVING MODEL---------------------
v0 = idm.v0; % desired velocity
T = idm.T; % Safe time headway
a = idm.a; % maximum acceleration
b = idm.b; %desired deceleration
delta = idm.delta; %acceleration exponent
s0 = idm.s0; % minimum distance
l = idm.l; % vehicle length
%============================================================

coolness = 0.99;          % coolness facotor


idx_observedcar = [];
idx_observing_mycar = [];
mycar.acceleration = 0;

FLAG_OTHERCAR_INTENTION_EST = 1;

    % if entering the plaza
if mycar.flgPlaza == 0 && mycar.pos(1) > 100*10^3
    mycar.flgPlaza = 1;
    
    mycar.pathTranslated = laneChangePath{mycar.goallane, mycar.save.lane_idx};
end

if mycar.flgPlaza == 0 % before entering plaza
    v0 = 15000;
    
elseif mycar.flgPlaza == 1 % after entering plaza
    pos = predict_pos(mycar.pos, mycar.vel, sim.T);
    mycar.targetDegree = get_tatgetTheta(pos,mycar.pathTranslated);
    mycar.vel(2) = (mycar.targetDegree - mycar.pos(3))/sim.T;
    
    if mycar.pos(1) < 187.5*10^3
        v0 = 15000;

    elseif mycar.pos(1) < 275*10^3
        v0 = 12500;
        
    elseif mycar.pos(1) < 320*10^3
        v0 = 10000;
    end
    
end



% estimate crashing othercar and identify approaching othercar in front of mycar----------------------
[idx_observedcar, t_observedcar, pos_mycarEst, pos_observedcarEst, idx_observing_mycar, angle_observing_mycar, mycarpos_observing_mycar] = is_carcrashed_orFollow_formycar_TTC_forwardrect(othercars, mycar, laneChangePath, FLAG_OTHERCAR_INTENTION_EST);


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
        
        if abs(pos_mycarEst(i,3) - pos_observedcarEst(i,3)) < 5
            FLAG_Follow_or_Cross = 1;
        else
            FLAG_Follow_or_Cross = 0;
        end
        
        %fprintf(1, 'after [%d] seconds, mycar and [%d](%d, %d) collide at (%d, %d)\n', t, idx_observedcar, othercars.car{idx_observedcar(i)}.pos(1), othercars.car{idx_observedcar(i)}.pos(2), pos_mycarEst(1), pos_mycarEst(2));
        
        %A3 = norm(othercars.car{idx_observedcar(i)}.pos(1:2) - mycar.pos(1:2));
        othercar_posEst_i = pos_observedcarEst(i,:);
        A3_TTC = norm(othercar_posEst_i(1:2) - mycar.pos(1:2)) - l;
        if A3_TTC < s0
            A3_TTC = s0;
        end

        if FLAG_Follow_or_Cross
            A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - (othercars.car{idx_observedcar(i)}.vel(1)*cos((othercars.car{idx_observedcar(i)}.pos(3)-mycar.pos(3))*pi/180)))/2/sqrt(a*b))/A3_TTC;
        else
            A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * mycar.vel(1)/2/sqrt(a*b))/A3_TTC;
        end
        
        A1 = mycar.vel(1)/v0;
        accele_IDM = a*(1 - A1^delta - A2^2);
        
        % -----ACC model-----------
        aLead = 0;  % this value need to be modified !!
        aLeadRestricted = min(aLead,a);
        
        if FLAG_Follow_or_Cross
            dvp = max(mycar.vel(1) - othercars.car{idx_observedcar(i)}.vel(1)*cos((othercars.car{idx_observedcar(i)}.pos(3)-mycar.pos(3))*pi/180), 0);
            vLead = othercars.car{idx_observedcar(i)}.vel(1)*cos((othercars.car{idx_observedcar(i)}.pos(3)-mycar.pos(3))*pi/180);
        else
            dvp = max(mycar.vel(1),0);
            vLead = 0;
        end
        
        denomCAH = vLead*vLead - 2*A3_TTC*aLeadRestricted;
        
        if (vLead*dvp < -2*A3_TTC*aLeadRestricted)&&(denomCAH~=0)
            accele_CAH = mycar.vel(1)*mycar.vel(1)*aLeadRestricted/denomCAH;
        else
            accele_CAH = aLeadRestricted - 0.5*dvp*dvp/max(A3_TTC,0.1);
        end
        
        if accele_IDM > accele_CAH
            cur_acceleration = accele_IDM;
        else
            cur_acceleration = (1-coolness)*accele_IDM + coolness*( accele_CAH + b*tanh((accele_IDM - accele_CAH)/b));
        end
        
        % -----end of ACC model---------------
        
        if i == 1
            min_acceleration = cur_acceleration;
        end
        
        if cur_acceleration <= min_acceleration
            mycar.acceleration = cur_acceleration;
            idx_maxDecelerate = idx_observedcar(i);
            t_maxDecelerate = t_observedcar(i);
        end
    end
    
    if FLAG_Follow_or_Cross
        if mycar.acceleration < -2940
            fprintf(2, 'mycar([%d, %d]) decelerate([%d]) to car [%d] (distance = [%d], observed time = [%d], reldegree = [%d]) by FOLLOW. Observed car position is [%d, %d]. A1=[%d], A2=[%d], A3_TTC=[%d]\n', mycar.pos(1), mycar.pos(2), mycar.acceleration, idx_maxDecelerate, A3_TTC, t_maxDecelerate, pos_mycarEst(i,3) - pos_observedcarEst(i,3), othercar_posEst_i(1), othercar_posEst_i(2), A1, A2, A3_TTC);
        else
            fprintf(1, 'mycar([%d, %d]) decelerate([%d]) to car [%d] (distance = [%d], observed time = [%d], reldegree = [%d]) by FOLLOW. Observed car position is [%d, %d]. A1=[%d], A2=[%d], A3_TTC=[%d]\n', mycar.pos(1), mycar.pos(2), mycar.acceleration, idx_maxDecelerate, A3_TTC, t_maxDecelerate, pos_mycarEst(i,3) - pos_observedcarEst(i,3), othercar_posEst_i(1), othercar_posEst_i(2), A1, A2, A3_TTC);
        end
    else
        if mycar.acceleration < -2940
            fprintf(2, 'mycar([%d, %d]) decelerate([%d]) to car [%d] (distance = [%d], observed time = [%d], reldegree = [%d]) by CROSS. Observed car position is [%d, %d]. A1=[%d], A2=[%d], A3_TTC=[%d]\n', mycar.pos(1), mycar.pos(2), mycar.acceleration, idx_maxDecelerate, A3_TTC, t_maxDecelerate, pos_mycarEst(i,3) - pos_observedcarEst(i,3), othercar_posEst_i(1), othercar_posEst_i(2), A1, A2, A3_TTC);
        else
            fprintf(1, 'mycar([%d, %d]) decelerate([%d]) to car [%d] (distance = [%d], observed time = [%d], reldegree = [%d]) by CROSS. Observed car position is [%d, %d]. A1=[%d], A2=[%d], A3_TTC=[%d]\n', mycar.pos(1), mycar.pos(2), mycar.acceleration, idx_maxDecelerate, A3_TTC, t_maxDecelerate, pos_mycarEst(i,3) - pos_observedcarEst(i,3), othercar_posEst_i(1), othercar_posEst_i(2), A1, A2, A3_TTC);
        end
    end
else
    A1 = mycar.vel(1)/v0;
    mycar.acceleration = a*(1 - A1^delta);
end



if ~isempty(idx_observing_mycar)
    % 自車を前方検知予測した他車の数分だけ、角度から交錯か追従か判断し、あるべき減速度を求め、それより早いorあるべき減速度が0.3Gを下回る場合、自社は譲る（一定減速度で減速）
    nr_observing_mycar = length(idx_observing_mycar);
    idx_maxDecelerate = [];
    t_maxDecelerate = [];
    min_acceleration = 0;
    
    % iterate by number of estimated collision cars (to calculate deceleration by TTC)
    for i = 1:nr_observing_mycar
        if angle_observing_mycar(i) < 5
            FLAG_Follow_or_Cross = 1;
        else
            FLAG_Follow_or_Cross = 0;
        end
        
        %fprintf(1, 'after [%d] seconds, mycar and [%d](%d, %d) collide at (%d, %d)\n', t, idx_observedcar, othercars.car{idx_observedcar(i)}.pos(1), othercars.car{idx_observedcar(i)}.pos(2), pos_mycarEst(1), pos_mycarEst(2));
        
        %A3 = norm(othercars.car{idx_observedcar(i)}.pos(1:2) - mycar.pos(1:2));
        mycar_posEst_i = mycarpos_observing_mycar(i,:);
        
        
        if FLAG_Follow_or_Cross
            % othercarsの成分に直して距離を取る
            [theta_other2mycar,~] = cart2pol(othercars.car{idx_observing_mycar(i)}.pos(1) - mycar.pos(1), othercars.car{idx_observing_mycar(i)}.pos(2) - mycar.pos(2));
            A3_TTC = norm(mycar.pos(1:2) - othercars.car{idx_observing_mycar(i)}.pos(1:2))*cos(theta_other2mycar*180/pi - othercars.car{idx_observing_mycar(i)}.pos(3)) - l;
            if A3_TTC < s0
                A3_TTC = s0;
            end
            A2 = (s0 + othercars.car{idx_observing_mycar(i)}.vel(1)*T + othercars.car{idx_observing_mycar(i)}.vel(1) * (othercars.car{idx_observing_mycar(i)}.vel(1) - (mycar.vel(1)*cos((mycar.pos(3)-othercars.car{idx_observing_mycar(i)}.pos(3))*pi/180)))/2/sqrt(a*b))/A3_TTC;
        else
            A3_TTC = norm(mycar_posEst_i(1:2) - othercars.car{idx_observing_mycar(i)}.pos(1:2)) - l;
            if A3_TTC < s0
                A3_TTC = s0;
            end
            A2 = (s0 + othercars.car{idx_observing_mycar(i)}.vel(1)*T + othercars.car{idx_observing_mycar(i)}.vel(1) * othercars.car{idx_observing_mycar(i)}.vel(1)/2/sqrt(a*b))/A3_TTC;
        end
        
        A1 = othercars.car{idx_observing_mycar(i)}.vel(1)/v0;
        accele_IDM = a*(1 - A1^delta - A2^2);
        
        % -----ACC model-----------
        aLead = 0;  % this value need to be modified !!
        aLeadRestricted = min(aLead,a);
        
        if FLAG_Follow_or_Cross
            dvp = max(othercars.car{idx_observing_mycar(i)}.vel(1) - mycar.vel(1),0);
            vLead = mycar.vel(1);
        else
            dvp = max(mycar.vel(1),0);
            vLead = 0;
        end
        
        denomCAH = vLead*vLead - 2*A3_TTC*aLeadRestricted;
        
        if (vLead*dvp < -2*A3_TTC*aLeadRestricted)&&(denomCAH~=0)
            accele_CAH = othercars.car{idx_observing_mycar(i)}.vel(1)*othercars.car{idx_observing_mycar(i)}.vel(1)*aLeadRestricted/denomCAH;
        else
            accele_CAH = aLeadRestricted - 0.5*dvp*dvp/max(A3_TTC,0.1);
        end
        
        if accele_IDM > accele_CAH
            moderate_acceleration = accele_IDM;
        else
            moderate_acceleration = (1-coolness)*accele_IDM + coolness*( accele_CAH + b*tanh((accele_IDM - accele_CAH)/b));
        end
        
        if FLAG_Follow_or_Cross
            fprintf(1, 'Car[%d]: moderate acceleration to mycar(distance = [%d], reldegree = [%d]) is [%d] by FOLLOW. Actual deceleration is [%d]\n', idx_observing_mycar(i), A3_TTC, angle_observing_mycar(i), moderate_acceleration, othercars.car{idx_observing_mycar(i)}.acceleration);
        else
            fprintf(1, 'Car[%d]: moderate acceleration to mycar(distance = [%d], reldegree = [%d]) is [%d] by CROSS. Actual deceleration is [%d]\n', idx_observing_mycar(i), A3_TTC, angle_observing_mycar(i), moderate_acceleration, othercars.car{idx_observing_mycar(i)}.acceleration);
            % if (moderate_acceleration < -2940 || othercars.car{idx_observing_mycar(i)}.acceleration > moderate_acceleration) && mycar.acceleration > -2940
            if othercars.car{idx_observing_mycar(i)}.acceleration > moderate_acceleration && mycar.acceleration > -2940
                mycar.acceleration = -2940;
                fprintf(2, 'mycar([%d, %d]) decelerate([%d]) to car [%d] as it doesnot slow down\n', mycar.pos(1), mycar.pos(2), mycar.acceleration, idx_observing_mycar(i));
            end
        end
        
        % -----end of ACC model---------------

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