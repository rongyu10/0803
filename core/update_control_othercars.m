function othercars = update_control_othercars(othercars, sim, idm, laneChangePath, lengthP)

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

for i = 1:othercars.n
    
    
%     if first_flag
%         %---- Control angular velocity: vel(2) --------
%         ratioSpeed = lengthP{othercars.car{i}.tolllane, othercars.car{i}.save.lane_idx}/(175*10^3);
%         othercars.car{i}.vel(1) = othercars.car{i}.vel(1)*ratioSpeed;
%         othercars.car{i}.pathTranslated = laneChangePath{othercars.car{i}.tolllane, othercars.car{i}.save.lane_idx};
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
        if othercars.car{i}.pos(1) > 100*10^3 && othercars.car{i}.pos(1) < 275*10^3
            othercars.car{i}.flgPlaza = 1;
            
            ratioSpeed = lengthP{othercars.car{i}.tolllane, othercars.car{i}.save.lane_idx}/(175*10^3);
            othercars.car{i}.vel(1) = othercars.car{i}.vel(1)*ratioSpeed;
            %othercars.car{i}.pathTranslated = update_laneChangePath(othercars.car{i},laneChangePath);
            %othercars.car{i}.pathTranslated = laneChangePath{othercars.car{i}.tolllane};
            othercars.car{i}.pathTranslated = laneChangePath{othercars.car{i}.tolllane, othercars.car{i}.save.lane_idx};
        end
    elseif othercars.car{i}.flgPlaza == 1
        
        
        %---- Control angular velocity: vel(2) --------
        pos = predict_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T);
        targetDegree = get_tatgetTheta(pos,othercars.car{i}.pathTranslated);
        if targetDegree - othercars.car{i}.pos(3) > 10
            othercars.car{i}.vel(2) = othercars.car{i}.vel(2) + 10/sim.T;
        elseif targetDegree - othercars.car{i}.pos(3) < -10
            othercars.car{i}.vel(2) = othercars.car{i}.vel(2) - 10/sim.T;
        else
            othercars.car{i}.vel(2) = othercars.car{i}.vel(2) + (targetDegree - othercars.car{i}.pos(3))/sim.T;
        end
        %----------------------------------------------
        
        othercars.car{i}.pos = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T);
        othercars.car{i}.bd  = get_carshape(othercars.car{i}.pos, othercars.car{i}.W, othercars.car{i}.H);
        
        if othercars.car{i}.pos(1) > 275*10^3 && othercars.car{i}.vel(1) > 10000
            othercars.car{i}.vel(1) = othercars.car{i}.vel(1) - 200;
        end
        
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