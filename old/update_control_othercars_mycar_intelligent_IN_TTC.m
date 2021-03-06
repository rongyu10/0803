function othercars = update_control_othercars_mycar_intelligent_IN_TTC(othercars, sim, mycar, idm, laneChangePath, lengthP, FLAG_LANECHANGE)

% PARAMETER OF INTELLIGENT DRIVING MODEL---------------------
v0 = idm.v0; % desired velocity
T = idm.T; % Safe time headway
a = idm.a; % maximum acceleration
b = idm.b; %desired deceleration
delta = idm.delta; %acceleration exponent
s0 = idm.s0; % minimum distance
l = idm.l; % vehicle length
%============================================================

for i = 1:othercars.n
    
    othercars.car{i}.est{1}.pos = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, 1);
    othercars.car{i}.est{2}.pos = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, 2);
    othercars.car{i}.est{3}.pos = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, 3);
    
    othercars.car{i}.est{1}.bd  = get_carshape(othercars.car{i}.est{1}.pos, othercars.car{i}.W, othercars.car{i}.H);
    othercars.car{i}.est{2}.bd  = get_carshape(othercars.car{i}.est{2}.pos, othercars.car{i}.W, othercars.car{i}.H);
    othercars.car{i}.est{3}.bd  = get_carshape(othercars.car{i}.est{3}.pos, othercars.car{i}.W, othercars.car{i}.H);
    
    if othercars.car{i}.flgPlaza == 0 % before entering plaza
        othercars.car{i}.pos ...
            = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T);
        othercars.car{i}.bd ...
            = get_carshape(othercars.car{i}.pos ...
            , othercars.car{i}.W, othercars.car{i}.H);
        
        
%         for t = 1:3
%             idx_crashcar = is_carcrashed_TTC(othercars, i, t);
%             predict collision by TTC (after 1,2,3(s))
%             if ~isempty(idx_crashcar)
%                 nr_crashcar = length(idx_crashcar);
%                 for j = 1:nr_crashcar
%                     if othercars.car{i}.pos(1) < othercars.car{idx_crashcar(nr_crashcar)}.pos(1)
%                         othercars.car{i}.vel(1) = othercars.car{i}.vel(1) - 5000*t*sim.T;
%                         break;
%                     end
%                 end
%                 break;
%             end
%         end
        
        
        % if entering the plaza
        if othercars.car{i}.pos(1) > 100*10^3 && othercars.car{i}.pos(1) < 275*10^3
            othercars.car{i}.flgPlaza = 1;
            
            ratioSpeed = lengthP{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx}/(175*10^3);
            othercars.car{i}.vel(1) = othercars.car{i}.vel(1)*ratioSpeed;
            othercars.car{i}.pathTranslated = laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx};
            
            % set the index number among the same target lane
            if isempty(find(othercars.car_nr(othercars.car{i}.goallane,:), 1, 'last')) % if there is no othercars heading for same lane
                idx_nr = 1;
            else
                idx_nr = find(othercars.car_nr(othercars.car{i}.goallane,:), 1, 'last') + 1;
            end
            
            % set the carID to the array
            othercars.car_nr(othercars.car{i}.goallane, idx_nr) = i;
        end
    elseif othercars.car{i}.flgPlaza == 1 % after entering plaza
        
        pos = predict_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T);
        targetDegree = get_tatgetTheta(pos,othercars.car{i}.pathTranslated);
        othercars.car{i}.pos(3) = targetDegree;
%         if targetDegree - othercars.car{i}.pos(3) > 5
%             othercars.car{i}.vel(2) = othercars.car{i}.vel(2) + 5/sim.T*0.01;
%         elseif targetDegree - othercars.car{i}.pos(3) < -5
%             othercars.car{i}.vel(2) = othercars.car{i}.vel(2) - 5/sim.T*0.01;
%         else
%             othercars.car{i}.vel(2) = othercars.car{i}.vel(2) + (targetDegree - othercars.car{i}.pos(3))/sim.T*0.01;
%         end
        %----------------------------------------------
        
        if i == mycar.rear_nr % IDM following mycar
            A3 = mycar.pos(1) - othercars.car{i}.pos(1) - l;
            A1 = othercars.car{i}.vel(1)/v0;
            A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - mycar.vel(1))/2/sqrt(a*b))/A3;
            
            if FLAG_LANECHANGE == 1
                fprintf(1, 'acceleration = [%4d] \n', a*(1 - A1^delta - A2^2));
                if a*(1 - A1^delta - A2^2) < - 1470
                    othercars.car{i}.angry = 1;
                else
                    othercars.car{i}.angry = 0;
                end
            end
            
            othercars.car{i}.vel(1) = othercars.car{i}.vel(1) + a*(1 - A1^delta - A2^2)*sim.T;
        else  % IDM following othercars
            other_front_nr = find(othercars.car_nr(othercars.car{i}.goallane,:) == i) - 1;
            
            if i ~= othercars.car_nr(othercars.car{i}.goallane,1) % IDM following frontcar
                A3 = othercars.car{othercars.car_nr(othercars.car{i}.goallane,other_front_nr)}.pos(1) - othercars.car{i}.pos(1) - l;
                A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - othercars.car{othercars.car_nr(othercars.car{i}.goallane,other_front_nr)}.vel(1))/2/sqrt(a*b))/A3;
            else
                A2 = 0;
            end
            A1 = othercars.car{i}.vel(1)/v0;
            othercars.car{i}.vel(1) = othercars.car{i}.vel(1) + a*(1 - A1^delta - A2^2)*sim.T;
        end
        
        for t = 1:3
            idx_crashcar = is_carcrashed_TTC(othercars, i, t);
            % predict collision by TTC (after 1,2,3(s))
            if ~isempty(idx_crashcar)
                fprintf(1, 'after [%d] seconds, [%d] and [%d] collide\n',t,i,idx_crashcar);
                nr_crashcar = length(idx_crashcar);
                for j = 1:nr_crashcar
                    if (othercars.car{i}.crossflg == 1 && othercars.car{j}.crossflg == 1) || (othercars.car{i}.crossflg == 0 && othercars.car{j}.crossflg == 0)
                        if othercars.car{i}.pos(1) < othercars.car{idx_crashcar(nr_crashcar)}.pos(1)
                            othercars.car{i}.vel(1) = othercars.car{i}.vel(1) - 5000*(4-t)*sim.T;
                            break;
                        end
                    elseif othercars.car{i}.crossflg == 1
                        othercars.car{i}.vel(1) = othercars.car{i}.vel(1) - 15000*(4-t)*sim.T;
                        break;
                    end
                end
                break;
            end
        end
        
        
        if othercars.car{i}.vel(1) < 0
            othercars.car{i}.vel(1) = 0;
        end
        
        if othercars.car{i}.vel(1) > 10000
            othercars.car{i}.vel(1) = othercars.car{i}.vel(1) - 2000 * sim.T; % decelerate if exceeds 36km/h in the plaza
        elseif othercars.car{i}.pos(1) > 275*10^3 && othercars.car{i}.vel(1) > 5000
            othercars.car{i}.vel(1) = othercars.car{i}.vel(1) - 4000 * sim.T; % decelerate if exceeds 18km/h in the tolllane
        end
        
        othercars.car{i}.pos = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T);
        othercars.car{i}.bd  = get_carshape(othercars.car{i}.pos, othercars.car{i}.W, othercars.car{i}.H);
        
    end
    
    
    
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