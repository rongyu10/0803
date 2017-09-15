function [mycar, othercars] = update_control_mycar_IN_IDMallandTTC_norfs_ACC3(mycar, sim, othercars, idm, laneChangePath)

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
time_TTC = 5.0;
step_TTC = 0.1;

idx_crashcar = [];

if mycar.flgPlaza == 0 % before entering plaza
    v0 = 15000;
    mycar.acceleration = 0;
    mycar.targetDegree_pre = 0;
    mycar.targetDegree = 0;
    mycar.targetDegree_dif = zeros(1, 5);
    mycar.targetDegree_dif_sum = 0;
    
elseif mycar.flgPlaza == 1 % after entering plaza
    pos = predict_pos(mycar.pos, mycar.vel, sim.T);
    mycar.targetDegree_pre = mycar.targetDegree;
    mycar.targetDegree = get_tatgetTheta(pos,mycar.pathTranslated);
    mycar.targetDegree_dif_sum = 0;
    
    for i = 1:4
        mycar.targetDegree_dif(i) = mycar.targetDegree_dif(i+1);
        mycar.targetDegree_dif_sum = mycar.targetDegree_dif_sum + mycar.targetDegree_dif(i);
    end
    
    mycar.targetDegree_dif(5) = mycar.targetDegree - mycar.targetDegree_pre;
    mycar.targetDegree_dif_sum = mycar.targetDegree_dif_sum + mycar.targetDegree_dif(5);
    
    mycar.vel(2) = mycar.targetDegree_dif_sum/sim.T/5;
    %fprintf(1, 'mycar.vel(2) = [%4d]', mycar.vel(2));
    mycar.pos(3) = mycar.targetDegree;
    
    if mycar.pos(1) < 187.5*10^3
        v0 = 12500;
        
    elseif mycar.pos(1) < 275*10^3
        v0 = 10000;
        if mycar.flgIDM == 0
            
            % set the index number among the same target lane
            if isempty(find(othercars.car_nr(mycar.selectlane,:), 1, 'last')) % if there is no othercars heading for same lane
                %mycar.front_nr = 0;
                idx_nr = 1;
            else
                %mycar.front_nr = othercars.car_nr(mycar.selectlane, find(othercars.car_nr(mycar.selectlane,:), 1, 'last'));
                idx_nr = find(othercars.car_nr(mycar.selectlane,:), 1, 'last') + 1;
            end
            othercars.car_nr(mycar.selectlane, idx_nr) = -1;
            mycar.flgIDM = 1;
        end
        
    elseif mycar.pos(1) < 320*10^3
        v0 = 7500;
    end
    
end

% estimate crashing othercar and identify approaching othercar in front of mycar----------------------
[idx_crashcar, t, mycar_posEst, idx_frontCar] = is_carcrashed_formycar_TTC_verIDM_inpol4_mycar(othercars, time_TTC, step_TTC, mycar);

nr_frontCar = length(idx_frontCar);
idx_mindist = [];
dist_min = 30000;
% ----------------------------------------------------------------------------------------------------

% make square of detecting area-----
for i = 0:10
    mycar.squareX(i+1) = mycar.pos(1) + 3000 * i;
    mycar.squareX(22-i) = mycar.squareX(i+1);
    if mycar.squareX(i+1) > 100*10^3 && mycar.squareX(i+1) < 275*10^3
        
        nData = size(laneChangePath{mycar.selectlane, mycar.save.lane_idx},1);
        for idx = 1:nData
            if mycar.squareX(i+1) - laneChangePath{mycar.selectlane, mycar.save.lane_idx}(idx,1) < 0
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
        
        left_right_point = get_car_futurepoint(mycar_posEst_det, mycar.W, mycar.H + 2500);
        mycar.squareX(i+1) = left_right_point(1,1);
        mycar.squareY(i+1) = left_right_point(1,2);
        mycar.squareX(22-i) = left_right_point(2,1);
        mycar.squareY(22-i) = left_right_point(2,2);
        
    else
        mycar.squareY(i+1) = mycar.pos(2) - 2500;
        mycar.squareY(22-i) = mycar.pos(2) + 2500;
    end
    
end
mycar.squareX(23) = mycar.squareX(1);
mycar.squareY(23) = mycar.squareY(1);
% ----------------------------------

% identify the nearest othercar in the detecting area --------------
for i = 1:nr_frontCar
    in = inpolygon(othercars.car{idx_frontCar(i)}.pos(1), othercars.car{idx_frontCar(i)}.pos(2), mycar.squareX, mycar.squareY);
    if in == 1
        if norm(mycar.pos(1:2) - othercars.car{idx_frontCar(i)}.pos(1:2)) < dist_min
            dist_min = norm(mycar.pos(1:2) - othercars.car{idx_frontCar(i)}.pos(1:2));
            idx_mindist = idx_frontCar(i);
        end
    end
end
% ------------------------------------------------------------------

if ~isempty(idx_crashcar)
    
    fprintf(1, 'after [%d] seconds, mycar and [%d](%d, %d) collide\n', t, idx_crashcar, othercars.car{idx_crashcar}.pos(1), othercars.car{idx_crashcar}.pos(2));
    
    A3 = norm(mycar_posEst(1:2) - mycar.pos(1:2));
    if A3 < 4000
        A3 = 4000;
    end
    A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{idx_crashcar}.vel(1))/2/sqrt(a*b))/A3;
    A1 = mycar.vel(1)/v0;
    mycar.acceleration = a*(1 - A1^delta - A2^2);
    
    if ~isempty(idx_mindist) % IDM following frontcar in the observing box
        fprintf(1, 'Following car [%d](%d, %d) by IDM\n',idx_mindist, othercars.car{idx_mindist}.pos(1), othercars.car{idx_mindist}.pos(2));
        A1 = mycar.vel(1)/v0;
        A3 = norm(othercars.car{idx_mindist}.pos(1:2) - mycar.pos(1:2)) - l;
        A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{idx_mindist}.vel(1))/2/sqrt(a*b))/A3;
        accele_IDM = a*(1 - A1^delta - A2^2);
        
        % -----ACC model-----------
        aLead = 0;  % this value need to be modified !!
        aLeadRestricted = min(aLead,a);
        dvp = max(mycar.vel(1) - othercars.car{idx_mindist}.vel(1),0);
        vLead = othercars.car{idx_mindist}.vel(1);
        
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
        
        if accele_ACC < mycar.acceleration
            mycar.acceleration = accele_ACC;
            fprintf(1, 'calculated by IDM is larger deceleration\n');
        end
    end
    
else
    if ~isempty(idx_mindist) % IDM following frontcar
        fprintf(1, 'Following car [%d](%d, %d) by IDM\n',idx_mindist, othercars.car{idx_mindist}.pos(1), othercars.car{idx_mindist}.pos(2));
        A1 = mycar.vel(1)/v0;
        A3 = norm(othercars.car{idx_mindist}.pos(1:2) - mycar.pos(1:2)) - l;
        A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{idx_mindist}.vel(1))/2/sqrt(a*b))/A3;
        
        accele_IDM = a*(1 - A1^delta - A2^2);
        
        % -----ACC model-----------
        aLead = 0;  % this value need to be modified !!
        aLeadRestricted = min(aLead,a);
        dvp = max(mycar.vel(1) - othercars.car{idx_mindist}.vel(1),0);
        vLead = othercars.car{idx_mindist}.vel(1);
        
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
        mycar.acceleration = accele_ACC;
        
    else
        A1 = mycar.vel(1)/v0;
        mycar.acceleration = a*(1 - A1^delta);
    end
    
end


mycar.vel(1) = mycar.vel(1) + mycar.acceleration*sim.T;


% control minimum velocity
if mycar.vel(1) < 0 && mycar.pos(1) < 320 * 10^3
    mycar.vel(1) = 0;
end

% UPDATE MY CAR INFORMATION
mycar.pos = update_pos(mycar.pos, mycar.vel, sim.T);
mycar.bd  = get_carshape(mycar.pos, mycar.W, mycar.H);

if mycar.acceleration < -4900
    fprintf(2, 'mycar(%d, %d) acceleration = [%4d] \n', mycar.pos(1), mycar.pos(2), mycar.acceleration);
    %mycar.acceleration = -9800;
else
    %fprintf(1, 'mycar(%d, %d) acceleration = [%4d] \n', mycar.pos(1), mycar.pos(2), mycar.acceleration);
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