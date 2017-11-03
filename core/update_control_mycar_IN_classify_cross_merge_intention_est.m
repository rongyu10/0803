function [mycar, table_same_lane] = update_control_mycar_IN_classify_cross_merge_intention_est(mycar, sim, othercars, idm, laneChangePath, table_same_lane, PLOT_MYCAR_DETECTING_AREA)


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

% PARAMETER OF TTC-------------------------------------------
time_TTC = 3.0;
step_TTC = sim.T;

idx_observedcar = [];

if mycar.flgPlaza == 0 % before entering plaza
    v0 = 15000;
%     mycar.acceleration = 0;
%     mycar.targetDegree_pre = 0;
%     mycar.targetDegree = 0;
%     mycar.targetDegree_dif = zeros(1, 5);
%     mycar.targetDegree_dif_sum = 0;
    
elseif mycar.flgPlaza == 1 % after entering plaza
    pos = predict_pos(mycar.pos, mycar.vel, sim.T);
    mycar.targetDegree = get_tatgetTheta(pos,mycar.pathTranslated);
    if mycar.targetDegree - mycar.pos(3) > 10
        mycar.vel(2) = mycar.vel(2) + 10/sim.T;
    elseif mycar.targetDegree - mycar.pos(3) < -10
        mycar.vel(2) = mycar.vel(2) - 10/sim.T;
    else
        mycar.vel(2) = mycar.vel(2) + (mycar.targetDegree - mycar.pos(3))/sim.T;
    end
    
    
    
    
%     
%     pos = predict_pos(mycar.pos, mycar.vel, sim.T);
%     mycar.targetDegree_pre = mycar.targetDegree;
%     mycar.targetDegree = get_tatgetTheta(pos,mycar.pathTranslated);
%     mycar.targetDegree_dif_sum = 0;
%     
%     for i = 1:4
%         mycar.targetDegree_dif(i) = mycar.targetDegree_dif(i+1);
%         mycar.targetDegree_dif_sum = mycar.targetDegree_dif_sum + mycar.targetDegree_dif(i);
%     end
%     
%     mycar.targetDegree_dif(5) = mycar.targetDegree - mycar.targetDegree_pre;
%     mycar.targetDegree_dif_sum = mycar.targetDegree_dif_sum + mycar.targetDegree_dif(5);
%     
%     mycar.vel(2) = mycar.targetDegree_dif_sum/sim.T/5;
%     mycar.pos(3) = mycar.targetDegree;
    
    
    
    
    
    if mycar.pos(1) < 187.5*10^3
        v0 = 15000;

    elseif mycar.pos(1) < 275*10^3
        v0 = 12500;
        if mycar.flgIDM == 0
            
            % set the index number among the same target lane
            if isempty(find(table_same_lane(mycar.selectlane,:), 1, 'last')) % if there is no othercars heading for same lane
                idx_nr = 1;
            else
                idx_nr = find(table_same_lane(mycar.selectlane,:), 1, 'last') + 1;
            end
            table_same_lane(mycar.selectlane, idx_nr) = -1; % idx '-1' indicates mycar
            mycar.flgIDM = 1;
        end
        
    elseif mycar.pos(1) < 320*10^3
        v0 = 10000;
    end
    
end



% estimate crashing othercar and identify approaching othercar in front of mycar----------------------
[idx_observedcar, t_observedcar, pos_mycarEst, pos_observedcarEst] = is_carcrashed_orFollow_formycar_TTC_forwardrect(othercars, time_TTC, step_TTC, mycar, laneChangePath);

if PLOT_MYCAR_DETECTING_AREA
    % make square of detecting area for drawing-----
    for i = 0:5
        mycar.squareX(i+1) = mycar.pos(1) + mycar.vel(1)*0.6*i;
        mycar.squareX(12-i) = mycar.squareX(i+1);
        
        if mycar.squareX(i+1) <= 100*10^3
            mycar.squareY(i+1) = mycar.pos(2) - 2500;
            mycar.squareY(12-i) = mycar.pos(2) + 2500;
            
        elseif mycar.squareX(i+1) <= 275*10^3
            nData = size(laneChangePath{mycar.selectlane, mycar.save.lane_idx},1);
            for idx = 1:nData
                % if mycar.squareX(i+1) - laneChangePath{mycar.selectlane, mycar.save.lane_idx}(idx,1) < 0
                if mycar.squareX(i+1) - 100*10^3 - 175/200*(idx - 1)*10^3 < 0
                    break;
                end
            end
            
            if idx~=nData
                vx= laneChangePath{mycar.selectlane, mycar.save.lane_idx}(idx+1,1)-laneChangePath{mycar.selectlane, mycar.save.lane_idx}(idx,1);
                vy= laneChangePath{mycar.selectlane, mycar.save.lane_idx}(idx+1,2)-laneChangePath{mycar.selectlane, mycar.save.lane_idx}(idx,2);
            else
                vx= laneChangePath{mycar.selectlane, mycar.save.lane_idx}(idx,1)-laneChangePath{mycar.selectlane, mycar.save.lane_idx}(idx-1,1);
                vy= laneChangePath{mycar.selectlane, mycar.save.lane_idx}(idx,2)-laneChangePath{mycar.selectlane, mycar.save.lane_idx}(idx-1,2);
            end
            
            mycar_posEst_det(1) = laneChangePath{mycar.selectlane, mycar.save.lane_idx}(idx,1);
            mycar_posEst_det(2) = laneChangePath{mycar.selectlane, mycar.save.lane_idx}(idx,2);
            mycar_posEst_det(3) = atan(vy/vx)*180/pi;
            
            left_right_point = get_car_futurepoint(mycar_posEst_det, mycar.W, 5000);
            mycar.squareX(i+1) = left_right_point(1,1);
            mycar.squareY(i+1) = left_right_point(1,2);
            mycar.squareX(12-i) = left_right_point(2,1);
            mycar.squareY(12-i) = left_right_point(2,2);
            
        else
            mycar.squareY(i+1) = (77.5-mycar.selectlane*5.0)*10^3 - 2500;
            mycar.squareY(12-i) = (77.5-mycar.selectlane*5.0)*10^3 + 2500;
            
        end
        
    end
    mycar.squareX(13) = mycar.squareX(1);
    mycar.squareY(13) = mycar.squareY(1);
    % ----------------------------------
end


if ~isempty(idx_observedcar)
    
    nr_observedCar = length(idx_observedcar);
    idx_maxDecelerate = [];
    t_maxDecelerate = [];
    min_acceleration = 0;
    
    % iterate by number of estimated collision cars (to calculate deceleration by TTC)
    for i = 1:nr_observedCar
        
        if abs(pos_mycarEst(3) - pos_observedcarEst(3)) < 5
            FLAG_Follow_or_Cross = 1;
        else
            FLAG_Follow_or_Cross = 0;
        end
        
        %fprintf(1, 'after [%d] seconds, mycar and [%d](%d, %d) collide at (%d, %d)\n', t, idx_observedcar, othercars.car{idx_observedcar(i)}.pos(1), othercars.car{idx_observedcar(i)}.pos(2), pos_mycarEst(1), pos_mycarEst(2));
        
        %A3 = norm(othercars.car{idx_observedcar(i)}.pos(1:2) - mycar.pos(1:2));
        othercar_posEst_i = pos_observedcarEst(i,:);
        A3_TTC = norm(othercar_posEst_i(1:2) - mycar.pos(1:2)) - l;


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
            dvp = max(mycar.vel(1) - othercars.car{idx_observedcar(i)}.vel(1),0);
            vLead = othercars.car{idx_observedcar(i)}.vel(1);
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
        fprintf(1, 'mycar decelerate to car [%d] (distance = [%d], observed time = [%d], reldegree = [%d]) by FOLLOW. Observed car position is [%d, %d]. A1=[%d], A2=[%d], A3_TTC=[%d]\n', idx_maxDecelerate, A3_TTC, t_maxDecelerate, pos_mycarEst(3) - pos_observedcarEst(3), othercar_posEst_i(1), othercar_posEst_i(2), A1, A2, A3_TTC);
    else
        fprintf(1, 'mycar decelerate to car [%d] (distance = [%d], observed time = [%d], reldegree = [%d]) by CROSS. Observed car position is [%d, %d]. A1=[%d], A2=[%d], A3_TTC=[%d]\n', idx_maxDecelerate, A3_TTC, t_maxDecelerate, pos_mycarEst(3) - pos_observedcarEst(3), othercar_posEst_i(1), othercar_posEst_i(2), A1, A2, A3_TTC);
    end
else
    A1 = mycar.vel(1)/v0;
    mycar.acceleration = a*(1 - A1^delta);
end

mycar.vel(1) = mycar.vel(1) + mycar.acceleration*sim.T;


% control minimum velocity
if mycar.vel(1) < 0 && mycar.pos(1) < 320 * 10^3
    mycar.vel(1) = 0;
end

% UPDATE MY CAR INFORMATION
mycar.pos = update_pos(mycar.pos, mycar.vel, sim.T);
mycar.bd  = get_carshape(mycar.pos, mycar.W, mycar.H);

if mycar.acceleration < -2940
    fprintf(2, 'mycar(%d, %d, %d) acceleration = [%4d] \n', mycar.pos(1), mycar.pos(2), mycar.pos(3), mycar.acceleration);
    %mycar.acceleration = -9800;
else
    fprintf(1, 'mycar(%d, %d, %d) acceleration = [%4d] \n', mycar.pos(1), mycar.pos(2), mycar.pos(3), mycar.acceleration);
end



% if entering the plaza
if mycar.pos(1) > 100*10^3 && mycar.pos(1) < 275*10^3
    mycar.flgPlaza = 1;
    
    mycar.pathTranslated = laneChangePath{mycar.selectlane, mycar.save.lane_idx};
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