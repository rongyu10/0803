function [othercars, table_same_lane] = update_control_othercars_mycar_IN_TTCandIDMall_manual_ref(othercars, sim, mycar, idm, laneChangePath, table_same_lane)


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
time_TTC = 3.0;
step_TTC = 0.1;

for i = 1:othercars.n
    
    if othercars.car{i}.flgPlaza == 0 % before entering plaza
        
        v0 = 15000;
        
        if othercars.car{i}.pos(1) > 0
            [idx_crashcar, t, mycar_posEst] = is_carcrashed_TTC_verIDM_widecar_4point_mycar2circle(othercars, i, time_TTC, step_TTC, mycar);
        end
        
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
            
        else
            A1 = othercars.car{i}.vel(1)/v0;
            othercars.car{i}.acceleration = a*(1 - A1^delta);
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
            v0 = 12500;
            [idx_crashcar, t, mycar_posEst] = is_carcrashed_TTC_verIDM_widecar_4point_mycar2circle(othercars, i, time_TTC, step_TTC, mycar);
            
            if ~isempty(idx_crashcar)
                
                if idx_crashcar == 0
                    %fprintf(1, 'after [%d] seconds, [%d] and mycar collide\n', t, i, idx_crashcar);
                else
                    %fprintf(1, 'after [%d] seconds, [%d] and [%d] collide\n', t, i, idx_crashcar);
                end
            
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
                A1 = othercars.car{i}.vel(1)/v0;
                othercars.car{i}.acceleration = a*(1 - A1^delta);
            end
            
        elseif othercars.car{i}.pos(1) < 320*10^3 
            v0 = 10000;
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
            
            [idx_crashcar, t, mycar_posEst] = is_carcrashed_TTC_verIDM_widecar_4point_mycar2circle(othercars, i, time_TTC, step_TTC, mycar);
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
                if i == mycar.rear_nr % IDM following mycar
                    A3 = norm(mycar.pos(1:2) - othercars.car{i}.pos(1:2)) - l;
                    A1 = othercars.car{i}.vel(1)/v0;
                    A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - mycar.vel(1))/2/sqrt(a*b))/A3;
                    
                    othercars.car{i}.acceleration = a*(1 - A1^delta - A2^2);
                    
                else  % IDM following othercars
                    other_front_nr = find(table_same_lane(othercars.car{i}.goallane,:) == i) - 1;
                    
                    if i ~= table_same_lane(othercars.car{i}.goallane,1) % IDM following frontcar
                        A3 = norm(othercars.car{table_same_lane(othercars.car{i}.goallane,other_front_nr)}.pos(1:2) - othercars.car{i}.pos(1:2)) - l;
                        A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - othercars.car{table_same_lane(othercars.car{i}.goallane,other_front_nr)}.vel(1))/2/sqrt(a*b))/A3;
                    else
                        A2 = 0;
                    end
                    A1 = othercars.car{i}.vel(1)/v0;
                    othercars.car{i}.acceleration = a*(1 - A1^delta - A2^2);
                end
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