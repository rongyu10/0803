function othercars = update_control_othercars_mycar_intelligent(othercars, sim, mycar, idm, laneChangePath, lengthP)

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
    if othercars.car{i}.flgPlaza == 0
        othercars.car{i}.pos ...
            = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T);
        othercars.car{i}.bd ...
            = get_carshape(othercars.car{i}.pos ...
            , othercars.car{i}.W, othercars.car{i}.H);
        if othercars.car{i}.pos(1) > 100*10^3 && othercars.car{i}.pos(1) < 275*10^3
            othercars.car{i}.flgPlaza = 1;
            
            ratioSpeed = lengthP{othercars.car{i}.tolllane, othercars.car{i}.save.lane_idx}/(175*10^3)*1.1;
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
        
%         if mod(i, othercars.npl) ~= 0
%             front_num = i + 1;
%             if mod(front_num, othercars.npl) == 1
%                 front_num = front_num - othercars.npl;
%             end
%             
%             if othercars.car{front_num}.pos(1) - othercars.car{i}.pos(1) < 0 % if there is no other car front of this car
%                 A3 = track.xmax - othercars.car{i}.pos(1) + othercars.car{front_num}.pos(1) - l;
%             else
%                 A3 = othercars.car{front_num}.pos(1) - othercars.car{i}.pos(1) - l;
%             end
%             A1 = othercars.car{i}.vel(1)/v0;
%             A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - othercars.car{front_num}.vel(1))/2/sqrt(a*b))/A3;
%             othercars.car{i}.vel(1) = othercars.car{i}.vel(1) + a*(1 - A1^delta - A2^2)*sim.T;
%             
%             if othercars.car{i}.vel(1) < 0
%                 othercars.car{i}.vel(1) = 0;
%             end
%         end
    
        
        
        % fprintf(1, 'mycar.pos(3) = [%4d] targetDegree = [%4d] mycar.vel(2) = [%4d] (targetDegree - pos(3))/sim.T = [%4d]\n', mycar.pos(3), targetDegree, mycar.vel(2), (targetDegree - pos(3))/sim.T);
        othercars.car{i}.pos = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T);
        othercars.car{i}.bd  = get_carshape(othercars.car{i}.pos, othercars.car{i}.W, othercars.car{i}.H);
        
        if othercars.car{i}.pos(1) > 275*10^3 && othercars.car{i}.vel(1) > 10000
            othercars.car{i}.vel(1) = othercars.car{i}.vel(1) - 200;
        end
        
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