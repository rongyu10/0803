function [othercars, table_same_lane] = update_control_othercars_mycar_IN_TTColdandIDM_IDM_ref(othercars, sim, mycar, idm, laneChangePath, table_same_lane)


% PARAMETER OF INTELLIGENT DRIVING MODEL---------------------
v0 = idm.v0; % desired velocity
T = idm.T; % Safe time headway
a = idm.a; % maximum acceleration
b = idm.b; %desired deceleration
delta = idm.delta; %acceleration exponent
s0 = idm.s0; % minimum distance
l = idm.l; % vehicle length
%============================================================

% PARAMETER OF TTC-------------------------------------------
time_TTC = 4.0;
step_TTC = 0.1;

coolness = 0.99;          % coolness facotor

for i = 1:othercars.n
    
    if othercars.car{i}.flgPlaza == 0 % before entering plaza
        
        v0 = 15000;
        
        if othercars.car{i}.pos(1) > 0
            [idx_nearCar, idx_crashcar, t, mycar_posEst, est_mycar] = is_carcrashed_TTC_verIDM_widecar_4point_mycar2circle(othercars, i, time_TTC, step_TTC, mycar);
        end
        
        squareX = zeros(1,13);
        squareY = zeros(1,13);
        % make square of detecting area for IDM-----
        for j = 0:5
            squareX(j+1) = othercars.car{i}.pos(1) + othercars.car{i}.vel(1)*0.6*j;
            squareX(12-j) = squareX(j+1);
            
            if squareX(j+1) <= 100*10^3
                squareY(j+1) = othercars.car{i}.pos(2) - 2500;
                squareY(12-j) = othercars.car{i}.pos(2) + 2500;
                
            elseif squareX(j+1) <= 275*10^3
                nData = size(laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx},1);
                for idx = 1:nData
                    if squareX(j+1) - laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx}(idx,1) < 0
                        break;
                    end
                end
                
                if idx~=nData
                    vx= laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx}(idx+1,1)-laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx}(idx,1);
                    vy= laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx}(idx+1,2)-laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx}(idx,2);
                else
                    vx= laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx}(idx,1)-laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx}(idx-1,1);
                    vy= laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx}(idx,2)-laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx}(idx-1,2);
                end
                
                mycar_posEst_det(1) = laneChangePath{othercars.car{i}.goallane, mycar.save.lane_idx}(idx,1);
                mycar_posEst_det(2) = laneChangePath{othercars.car{i}.goallane, mycar.save.lane_idx}(idx,2);
                mycar_posEst_det(3) = atan(vy/vx)*180/pi;
                
                left_right_point = get_car_futurepoint(mycar_posEst_det, mycar.W, 5000);
                squareX(j+1) = left_right_point(1,1);
                squareY(j+1) = left_right_point(1,2);
                squareX(12-j) = left_right_point(2,1);
                squareY(12-j) = left_right_point(2,2);
                
            else
                squareY(j+1) = (77.5-othercars.car{i}.goallane*5.0)*10^3 - 2500;
                squareY(12-j) = (77.5-othercars.car{i}.goallane*5.0)*10^3 + 2500;
                
            end
            
        end
        squareX(13) = squareX(1);
        squareY(13) = squareY(1);
        % ----------------------------------
        
        % identify the nearest othercar or mycar in the detecting area --------------
        nr_nearCar = length(idx_nearCar);
        idx_mindist = [];
        dist_min = 30000;
        for j = 1:nr_nearCar
            in = inpolygon(othercars.car{idx_nearCar(j)}.pos(1), othercars.car{idx_nearCar(j)}.pos(2), squareX, squareY);
            if in == 1
                if norm(othercars.car{i}.pos(1:2) - othercars.car{idx_nearCar(j)}.pos(1:2)) < dist_min
                    dist_min = norm(othercars.car{i}.pos(1:2) - othercars.car{idx_nearCar(j)}.pos(1:2));
                    idx_mindist = idx_nearCar(j);
                end
            end
        end
        
        if est_mycar == 1
            in = inpolygon(mycar.pos(1), mycar.pos(2), squareX, squareY);
            if in == 1
                if norm(othercars.car{i}.pos(1:2) - mycar.pos(1:2)) < dist_min
                    idx_mindist = 0;
                end
            end
        end
        
        % ------------------------------------------------------------------
        
        if ~isempty(idx_crashcar)
            
            %fprintf(1, 'after [%d] seconds, [%d] and [%d] collide\n', t, i, idx_crashcar);
            
            A3 = norm(mycar_posEst(1:2) - othercars.car{i}.pos(1:2));
            %A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * othercars.car{i}.vel(1)/2/sqrt(a*b))/A3;
            if idx_crashcar > 0
                A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - othercars.car{idx_crashcar}.vel(1))/2/sqrt(a*b))/A3;
            else
                A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - mycar.vel(1))/2/sqrt(a*b))/A3;
            end
            A1 = othercars.car{i}.vel(1)/v0;
            othercars.car{i}.acceleration = a*(1 - A1^delta - A2^2);
            
            
            if ~isempty(idx_mindist) % IDM following frontcar in the observing box
                %fprintf(1, 'Following car [%d](%d, %d) by IDM\n',idx_mindist, othercars.car{idx_mindist}.pos(1), othercars.car{idx_mindist}.pos(2));
                
                if idx_mindist == 0 % for mycar
                    A1 = othercars.car{i}.vel(1)/v0;
                    A3 = norm(mycar.pos(1:2) - othercars.car{i}.pos(1:2)) - l;
                    A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - (mycar.vel(1)*cos((mycar.pos(3)-othercars.car{i}.pos(3))*pi/180)))/2/sqrt(a*b))/A3;
                    accele_IDM = a*(1 - A1^delta - A2^2);
                    
                    % -----ACC model-----------
                    aLead = 0;  % this value need to be modified !!
                    aLeadRestricted = min(aLead,a);
                    dvp = max(othercars.car{i}.vel(1) - mycar.vel(1),0);
                    vLead = mycar.vel(1);
                else % for othercars
                    A1 = othercars.car{i}.vel(1)/v0;
                    A3 = norm(othercars.car{idx_mindist}.pos(1:2) - othercars.car{i}.pos(1:2)) - l;
                    A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - (othercars.car{idx_mindist}.vel(1)*cos((othercars.car{idx_mindist}.pos(3)-othercars.car{i}.pos(3))*pi/180)))/2/sqrt(a*b))/A3;
                    accele_IDM = a*(1 - A1^delta - A2^2);
                    
                    % -----ACC model-----------
                    aLead = 0;  % this value need to be modified !!
                    aLeadRestricted = min(aLead,a);
                    dvp = max(othercars.car{i}.vel(1) - othercars.car{idx_mindist}.vel(1),0);
                    vLead = othercars.car{idx_mindist}.vel(1);
                end
                
                denomCAH = vLead*vLead - 2*A3*aLeadRestricted;
                
                if (vLead*dvp < -2*A3*aLeadRestricted)&&(denomCAH~=0)
                    accele_CAH = v*v*aLeadRestricted/denomCAH;
                else
                    accele_CAH = aLeadRestricted - 0.5*dvp*dvp/max(A3,0.1);
                end
                
                if accele_IDM > accele_CAH
                    accele_ACC = accele_IDM;
                else
                    accele_ACC = (1-coolness)*accele_IDM + coolness*( accele_CAH + b*tanh((accele_IDM - accele_CAH)/b));
                end
                
                if accele_ACC < othercars.car{i}.acceleration
                    othercars.car{i}.acceleration = accele_ACC;
                    %fprintf(1, 'CAR[%d]: calculated by IDM to [%d] is larger deceleration\n', i, idx_mindist);
                else
                    %fprintf(1, 'CAR[%d]: calculated by TTC to [%d] is larger deceleration\n', i, idx_crashcar);
                end
            end
            
        else
            
            if ~isempty(idx_mindist) % IDM following frontcar in the observing box
                %fprintf(1, 'Following car [%d](%d, %d) by IDM\n',idx_mindist, othercars.car{idx_mindist}.pos(1), othercars.car{idx_mindist}.pos(2));
                if idx_mindist == 0 % for mycar
                    A1 = othercars.car{i}.vel(1)/v0;
                    A3 = norm(mycar.pos(1:2) - othercars.car{i}.pos(1:2)) - l;
                    A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - (mycar.vel(1)*cos((mycar.pos(3)-othercars.car{i}.pos(3))*pi/180)))/2/sqrt(a*b))/A3;
                    accele_IDM = a*(1 - A1^delta - A2^2);
                    
                    % -----ACC model-----------
                    aLead = 0;  % this value need to be modified !!
                    aLeadRestricted = min(aLead,a);
                    dvp = max(othercars.car{i}.vel(1) - mycar.vel(1),0);
                    vLead = mycar.vel(1);
                else % for othercars
                    A1 = othercars.car{i}.vel(1)/v0;
                    A3 = norm(othercars.car{idx_mindist}.pos(1:2) - othercars.car{i}.pos(1:2)) - l;
                    A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - (othercars.car{idx_mindist}.vel(1)*cos((othercars.car{idx_mindist}.pos(3)-othercars.car{i}.pos(3))*pi/180)))/2/sqrt(a*b))/A3;
                    accele_IDM = a*(1 - A1^delta - A2^2);
                    
                    % -----ACC model-----------
                    aLead = 0;  % this value need to be modified !!
                    aLeadRestricted = min(aLead,a);
                    dvp = max(othercars.car{i}.vel(1) - othercars.car{idx_mindist}.vel(1),0);
                    vLead = othercars.car{idx_mindist}.vel(1);
                end
                denomCAH = vLead*vLead - 2*A3*aLeadRestricted;
                
                if (vLead*dvp < -2*A3*aLeadRestricted)&&(denomCAH~=0)
                    accele_CAH = v*v*aLeadRestricted/denomCAH;
                else
                    accele_CAH = aLeadRestricted - 0.5*dvp*dvp/max(A3,0.1);
                end
                
                if accele_IDM > accele_CAH
                    accele_ACC = accele_IDM;
                else
                    accele_ACC = (1-coolness)*accele_IDM + coolness*( accele_CAH + b*tanh((accele_IDM - accele_CAH)/b));
                end
                
                othercars.car{i}.acceleration = accele_ACC;
                
                %fprintf(1, 'CAR[%d]: calculated by IDM to [%d] only\n', i, idx_mindist);
            else
                A1 = othercars.car{i}.vel(1)/v0;
                othercars.car{i}.acceleration = a*(1 - A1^delta);
            end
            
        end
        
        % if entering the plaza
        if othercars.car{i}.pos(1) > 100*10^3 && othercars.car{i}.pos(1) < 275*10^3
            othercars.car{i}.flgPlaza = 1;

            othercars.car{i}.pathTranslated = laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx};
            
        end
    elseif othercars.car{i}.flgPlaza == 1 % after entering plaza
        
        pos = predict_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T);
        targetDegree = get_tatgetTheta(pos,othercars.car{i}.pathTranslated);
        othercars.car{i}.pos(3) = targetDegree;
        
        if othercars.car{i}.pos(1) < 187.5*10^3
            v0 = 15000;
            [idx_nearCar, idx_crashcar, t, mycar_posEst, est_mycar] = is_carcrashed_TTC_verIDM_widecar_4point_mycar2circle(othercars, i, time_TTC, step_TTC, mycar);
            
            squareX = zeros(1,13);
            squareY = zeros(1,13);
            % make square of detecting area for IDM-----
            for j = 0:5
                squareX(j+1) = othercars.car{i}.pos(1) + othercars.car{i}.vel(1)*0.6*j;
                squareX(12-j) = squareX(j+1);
                
                if squareX(j+1) <= 100*10^3
                    squareY(j+1) = othercars.car{i}.pos(2) - 2500;
                    squareY(12-j) = othercars.car{i}.pos(2) + 2500;
                    
                elseif squareX(j+1) <= 275*10^3
                    nData = size(laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx},1);
                    for idx = 1:nData
                        if squareX(j+1) - laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx}(idx,1) < 0
                            break;
                        end
                    end
                    
                    if idx~=nData
                        vx= laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx}(idx+1,1)-laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx}(idx,1);
                        vy= laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx}(idx+1,2)-laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx}(idx,2);
                    else
                        vx= laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx}(idx,1)-laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx}(idx-1,1);
                        vy= laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx}(idx,2)-laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx}(idx-1,2);
                    end
                    
                    mycar_posEst_det(1) = laneChangePath{othercars.car{i}.goallane, mycar.save.lane_idx}(idx,1);
                    mycar_posEst_det(2) = laneChangePath{othercars.car{i}.goallane, mycar.save.lane_idx}(idx,2);
                    mycar_posEst_det(3) = atan(vy/vx)*180/pi;
                    
                    left_right_point = get_car_futurepoint(mycar_posEst_det, mycar.W, 5000);
                    squareX(j+1) = left_right_point(1,1);
                    squareY(j+1) = left_right_point(1,2);
                    squareX(12-j) = left_right_point(2,1);
                    squareY(12-j) = left_right_point(2,2);
                    
                else
                    squareY(j+1) = (77.5-othercars.car{i}.goallane*5.0)*10^3 - 2500;
                    squareY(12-j) = (77.5-othercars.car{i}.goallane*5.0)*10^3 + 2500;
                    
                end
                
            end
            squareX(13) = squareX(1);
            squareY(13) = squareY(1);
            % ----------------------------------
            
            % identify the nearest othercar or mycar in the detecting area --------------
            nr_nearCar = length(idx_nearCar);
            idx_mindist = [];
            dist_min = 30000;
            for j = 1:nr_nearCar
                in = inpolygon(othercars.car{idx_nearCar(j)}.pos(1), othercars.car{idx_nearCar(j)}.pos(2), squareX, squareY);
                if in == 1
                    if norm(othercars.car{i}.pos(1:2) - othercars.car{idx_nearCar(j)}.pos(1:2)) < dist_min
                        dist_min = norm(othercars.car{i}.pos(1:2) - othercars.car{idx_nearCar(j)}.pos(1:2));
                        idx_mindist = idx_nearCar(j);
                    end
                end
            end
            
            if est_mycar == 1
                in = inpolygon(mycar.pos(1), mycar.pos(2), squareX, squareY);
                if in == 1
                    if norm(othercars.car{i}.pos(1:2) - mycar.pos(1:2)) < dist_min
                        idx_mindist = idx_nearCar(j);
                    end
                end
            end
            % ------------------------------------------------------------------
            
            if ~isempty(idx_crashcar)
                
                %fprintf(1, 'after [%d] seconds, [%d] and [%d] collide\n', t, i, idx_crashcar);
                
                A3 = norm(mycar_posEst(1:2) - othercars.car{i}.pos(1:2));
                %A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * othercars.car{i}.vel(1)/2/sqrt(a*b))/A3;
                if idx_crashcar > 0
                    A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - othercars.car{idx_crashcar}.vel(1))/2/sqrt(a*b))/A3;
                else
                    A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - mycar.vel(1))/2/sqrt(a*b))/A3;
                end
                A1 = othercars.car{i}.vel(1)/v0;
                othercars.car{i}.acceleration = a*(1 - A1^delta - A2^2);
                
                
                if ~isempty(idx_mindist) % IDM following frontcar in the observing box
                    %fprintf(1, 'Following car [%d](%d, %d) by IDM\n',idx_mindist, othercars.car{idx_mindist}.pos(1), othercars.car{idx_mindist}.pos(2));
                    if idx_mindist == 0
                        A1 = othercars.car{i}.vel(1)/v0;
                        A3 = norm(mycar.pos(1:2) - othercars.car{i}.pos(1:2)) - l;
                        A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - (mycar.vel(1)*cos((mycar.pos(3)-othercars.car{i}.pos(3))*pi/180)))/2/sqrt(a*b))/A3;
                        accele_IDM = a*(1 - A1^delta - A2^2);
                        
                        % -----ACC model-----------
                        aLead = 0;  % this value need to be modified !!
                        aLeadRestricted = min(aLead,a);
                        dvp = max(othercars.car{i}.vel(1) - mycar.vel(1),0);
                        vLead = mycar.vel(1);
                    else
                        A1 = othercars.car{i}.vel(1)/v0;
                        A3 = norm(othercars.car{idx_mindist}.pos(1:2) - othercars.car{i}.pos(1:2)) - l;
                        A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - (othercars.car{idx_mindist}.vel(1)*cos((othercars.car{idx_mindist}.pos(3)-othercars.car{i}.pos(3))*pi/180)))/2/sqrt(a*b))/A3;
                        accele_IDM = a*(1 - A1^delta - A2^2);
                        
                        % -----ACC model-----------
                        aLead = 0;  % this value need to be modified !!
                        aLeadRestricted = min(aLead,a);
                        dvp = max(othercars.car{i}.vel(1) - othercars.car{idx_mindist}.vel(1),0);
                        vLead = othercars.car{idx_mindist}.vel(1);
                    end
                
                    denomCAH = vLead*vLead - 2*A3*aLeadRestricted;
                    
                    if (vLead*dvp < -2*A3*aLeadRestricted)&&(denomCAH~=0)
                        accele_CAH = v*v*aLeadRestricted/denomCAH;
                    else
                        accele_CAH = aLeadRestricted - 0.5*dvp*dvp/max(A3,0.1);
                    end
                    
                    if accele_IDM > accele_CAH
                        accele_ACC = accele_IDM;
                    else
                        accele_ACC = (1-coolness)*accele_IDM + coolness*( accele_CAH + b*tanh((accele_IDM - accele_CAH)/b));
                    end
                    
                    
                    if accele_ACC < othercars.car{i}.acceleration
                        othercars.car{i}.acceleration = accele_ACC;
                        %fprintf(1, 'CAR[%d]: calculated by IDM to [%d] is larger deceleration\n', i, idx_mindist);
                    else
                        %fprintf(1, 'CAR[%d]: calculated by TTC to [%d] is larger deceleration\n', i, idx_crashcar);
                    end
                    
                    
                end
                
            else
                
                if ~isempty(idx_mindist) % IDM following frontcar in the observing box
                    %fprintf(1, 'Following car [%d](%d, %d) by IDM\n',idx_mindist, othercars.car{idx_mindist}.pos(1), othercars.car{idx_mindist}.pos(2));
                    if idx_mindist == 0
                        A1 = othercars.car{i}.vel(1)/v0;
                        A3 = norm(mycar.pos(1:2) - othercars.car{i}.pos(1:2)) - l;
                        A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - (mycar.vel(1)*cos((mycar.pos(3)-othercars.car{i}.pos(3))*pi/180)))/2/sqrt(a*b))/A3;
                        accele_IDM = a*(1 - A1^delta - A2^2);
                        
                        % -----ACC model-----------
                        aLead = 0;  % this value need to be modified !!
                        aLeadRestricted = min(aLead,a);
                        dvp = max(othercars.car{i}.vel(1) - mycar.vel(1),0);
                        vLead = mycar.vel(1);
                    else
                        A1 = othercars.car{i}.vel(1)/v0;
                        A3 = norm(othercars.car{idx_mindist}.pos(1:2) - othercars.car{i}.pos(1:2)) - l;
                        A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - (othercars.car{idx_mindist}.vel(1)*cos((othercars.car{idx_mindist}.pos(3)-othercars.car{i}.pos(3))*pi/180)))/2/sqrt(a*b))/A3;
                        accele_IDM = a*(1 - A1^delta - A2^2);
                        
                        % -----ACC model-----------
                        aLead = 0;  % this value need to be modified !!
                        aLeadRestricted = min(aLead,a);
                        dvp = max(othercars.car{i}.vel(1) - othercars.car{idx_mindist}.vel(1),0);
                        vLead = othercars.car{idx_mindist}.vel(1);
                    end
                    
                    denomCAH = vLead*vLead - 2*A3*aLeadRestricted;
                    
                    if (vLead*dvp < -2*A3*aLeadRestricted)&&(denomCAH~=0)
                        accele_CAH = v*v*aLeadRestricted/denomCAH;
                    else
                        accele_CAH = aLeadRestricted - 0.5*dvp*dvp/max(A3,0.1);
                    end
                    
                    if accele_IDM > accele_CAH
                        accele_ACC = accele_IDM;
                    else
                        accele_ACC = (1-coolness)*accele_IDM + coolness*( accele_CAH + b*tanh((accele_IDM - accele_CAH)/b));
                    end
                    
                    othercars.car{i}.acceleration = accele_ACC;
                    
                    
                    %fprintf(1, 'CAR[%d]: calculated by IDM to [%d] only\n', i, idx_mindist);
                    
                    
                else
                    A1 = othercars.car{i}.vel(1)/v0;
                    othercars.car{i}.acceleration = a*(1 - A1^delta);
                end
                
            end
            
        elseif othercars.car{i}.pos(1) < 320*10^3 
            v0 = 12500;
            if othercars.car{i}.flgIDM == 0
                
                % set the index number among the same target lane
                if isempty(find(table_same_lane(othercars.car{i}.goallane,:), 1, 'last')) % if there is no othercars heading for same lane
                    idx_nr = 1;
                else
                    idx_nr = find(table_same_lane(othercars.car{i}.goallane,:), 1, 'last') + 1;
                    if table_same_lane(othercars.car{i}.goallane,idx_nr - 1) == -1
                        mycar.rear_nr = i;
                    end
                end
                
                % set the carID to the array
                table_same_lane(othercars.car{i}.goallane, idx_nr) = i;
                
                othercars.car{i}.flgIDM = 1;
            end
            
            [idx_nearCar, idx_crashcar, t, mycar_posEst, est_mycar] = is_carcrashed_TTC_verIDM_widecar_4point_mycar2circle(othercars, i, time_TTC, step_TTC, mycar);
            if ~isempty(idx_crashcar)
                
                A3 = norm(mycar_posEst(1:2) - othercars.car{i}.pos(1:2));
                %A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * othercars.car{i}.vel(1)/2/sqrt(a*b))/A3;
                if idx_crashcar > 0
                    A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - othercars.car{idx_crashcar}.vel(1))/2/sqrt(a*b))/A3;
                else
                    A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - mycar.vel(1))/2/sqrt(a*b))/A3;
                end
                A1 = othercars.car{i}.vel(1)/v0;
                othercars.car{i}.acceleration = a*(1 - A1^delta - A2^2);
                
            else
                other_front_nr = find(table_same_lane(othercars.car{i}.goallane,:) == i) - 1;
                
                if other_front_nr == 0 % if following nocar
                    A2 = 0;
                elseif table_same_lane(othercars.car{i}.goallane,other_front_nr) == -1 % if following mycar
                    A3 = norm(mycar.pos(1:2) - othercars.car{i}.pos(1:2)) - l;
                    A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - mycar.vel(1))/2/sqrt(a*b))/A3;
                else % if following othercars
                    A3 = norm(othercars.car{table_same_lane(othercars.car{i}.goallane,other_front_nr)}.pos(1:2) - othercars.car{i}.pos(1:2)) - l;
                    A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - othercars.car{table_same_lane(othercars.car{i}.goallane,other_front_nr)}.vel(1))/2/sqrt(a*b))/A3;
                end
                    
                A1 = othercars.car{i}.vel(1)/v0;
                othercars.car{i}.acceleration = a*(1 - A1^delta - A2^2);

            end
        end
    end
end

% update position after updating all the velocity of othercars
for i = 1:othercars.n
    othercars.car{i}.vel(1) = othercars.car{i}.vel(1) + othercars.car{i}.acceleration*sim.T;
    
    % decelerate in the toll lane
    if othercars.car{i}.pos(1) > 275*10^3 && othercars.car{i}.vel(1) > 5000
        othercars.car{i}.vel(1) = othercars.car{i}.vel(1) - 3000 * sim.T; % decelerate if exceeds 18km/h in the tolllane
    end
    
    % control minimum velocity
    if othercars.car{i}.vel(1) < 0 && othercars.car{i}.pos(1) < 320 * 10^3
        othercars.car{i}.vel(1) = 0;
    end
    
    othercars.car{i}.pos = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T);
    othercars.car{i}.bd  = get_carshape(othercars.car{i}.pos, othercars.car{i}.W, othercars.car{i}.H);
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