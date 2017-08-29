function othercars = update_control_othercars_mycar_intelligent_OUT_TTCtoIDM_ver2(othercars, sim, mycar, idm, laneChangePath, lengthP, FLAG_LANECHANGE)

% persistent first_flag
% if isempty(first_flag)
%     first_flag = true;
% end

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
time_TTC = 2.0;
step_TTC = 0.1;



for i = 1:othercars.n
    
    
    %     if first_flag
    %         %---- Control angular velocity: vel(2) --------
    %         ratioSpeed = lengthP{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx}/(175*10^3);
    %         othercars.car{i}.vel(1) = othercars.car{i}.vel(1)*ratioSpeed;
    %         othercars.car{i}.pathTranslated = laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx};
    %     end
    %
    %     pos = predict_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T);
    %     targetDegree = get_tatgetTheta(pos,othercars.car{i}.pathTranslated);
    %     if targetDegree - othercars.car{i}.pos(3) > 10
    %         othercars.car{i}.vel(2) = othercars.car{i}.vel(2) + 10/sim.T;
    %     elseif targetDegree - othercars.car{i}.pos(3) < -10
    %         othercars.car{i}.vel(2) = othercars.car{i}.vel(2) - 10/sim.T;
    %     else
    %         othercars.car{i}.vel(2) = othercars.car{i}.vel(2) + (targetDegree - othercars.car{i}.pos(3))/sim.T;
    %     end
    %     %----------------------------------------------
    %
    %     othercars.car{i}.pos = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T);
    %     othercars.car{i}.bd  = get_carshape(othercars.car{i}.pos, othercars.car{i}.W, othercars.car{i}.H);
    %
    %     if othercars.car{i}.pos(1) > 275*10^3 && othercars.car{i}.vel(1) > 10000
    %         othercars.car{i}.vel(1) = othercars.car{i}.vel(1) - 200;
    %     end
    
    if othercars.car{i}.flgPlaza == 0
        othercars.car{i}.pos ...
            = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T);
        othercars.car{i}.bd ...
            = get_carshape(othercars.car{i}.pos ...
            , othercars.car{i}.W, othercars.car{i}.H);
        if othercars.car{i}.pos(1) > 0
            othercars.car{i}.vel(1) = othercars.car{i}.vel(1) + 1000 * sim.T;
        end
        if othercars.car{i}.pos(1) > 45*10^3 && othercars.car{i}.pos(1) < 270*10^3
            othercars.car{i}.flgPlaza = 1;
            
            ratioSpeed = lengthP{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx}/(225*10^3);
            othercars.car{i}.vel(1) = othercars.car{i}.vel(1)*ratioSpeed;
            %othercars.car{i}.pathTranslated = update_laneChangePath(othercars.car{i},laneChangePath);
            %othercars.car{i}.pathTranslated = laneChangePath{othercars.car{i}.goallane};
            othercars.car{i}.pathTranslated = laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx};
            
        end
    elseif othercars.car{i}.flgPlaza == 1
        
        
        %---- Control angular velocity: vel(2) --------
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
        
        if othercars.car{i}.pos(1) < 240*10^3
            
            [idx_crashcar, t] = is_carcrashed_TTC_ver2(othercars, i, time_TTC, step_TTC);
            if ~isempty(idx_crashcar)
                fprintf(1, 'after [%d] seconds, [%d] and [%d] collide\n',t,i,idx_crashcar);
                nr_crashcar = length(idx_crashcar);
                for j = 1:nr_crashcar
%                     if (othercars.car{i}.crossflg == 1 && othercars.car{idx_crashcar(j)}.crossflg == 1) || (othercars.car{i}.crossflg == 0 && othercars.car{idx_crashcar(j)}.crossflg == 0)
%                         if othercars.car{i}.pos(1) < othercars.car{idx_crashcar(j)}.pos(1)
%                             dist = norm(othercars.car{i}.pos - othercars.car{idx_crashcar(j)}.pos);
%                             othercars.car{i}.vel(1) = othercars.car{i}.vel(1) - 10000000*(3-t)^2/dist*sim.T;
%                             break;
%                         end
%                     elseif othercars.car{i}.crossflg == 1
%                         dist = norm(othercars.car{i}.pos - othercars.car{idx_crashcar(j)}.pos);
%                         othercars.car{i}.vel(1) = othercars.car{i}.vel(1) - 10000000*(3-t)^2/dist*sim.T;
%                         break;
%                     end
                    if othercars.car{i}.pos(1) < othercars.car{idx_crashcar(j)}.pos(1)
                          dist = norm(othercars.car{i}.pos - othercars.car{idx_crashcar(j)}.pos);
                          othercars.car{i}.vel(1) = othercars.car{i}.vel(1) - 50000000/dist/t*sim.T;
                          break;
                    end
                    if j == nr_crashcar % if "othercars.car{i}" does not decelerate
                        A1 = othercars.car{i}.vel(1)/v0;
                        othercars.car{i}.vel(1) = othercars.car{i}.vel(1) + a*(1 - A1^delta)*sim.T;
                    end
                end
            else
                A1 = othercars.car{i}.vel(1)/v0;
                othercars.car{i}.vel(1) = othercars.car{i}.vel(1) + a*(1 - A1^delta)*sim.T;
            end
            
        else
            
            if othercars.car{i}.flgIDM == 0
                if isempty(find(othercars.car_nr(othercars.car{i}.goallane,:), 1, 'last'))
                    idx_nr = 1;
                else
                    idx_nr = find(othercars.car_nr(othercars.car{i}.goallane,:), 1, 'last') + 1;
                end
                othercars.car_nr(othercars.car{i}.goallane, idx_nr) = i;
                
                othercars.car{i}.flgIDM = 1;
            end
            
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
            else
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
            
        end
        
        
        if othercars.car{i}.vel(1) < 5000
            othercars.car{i}.vel(1) = 5000;
        end
        
        othercars.car{i}.pos = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T);
        othercars.car{i}.bd  = get_carshape(othercars.car{i}.pos, othercars.car{i}.W, othercars.car{i}.H);
        
    end
    
    
    
end

% if first_flag
%     first_flag = false;
% end

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