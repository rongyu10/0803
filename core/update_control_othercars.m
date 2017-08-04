function othercars = update_control_othercars(othercars, sim)

for i = 1:othercars.n
    if othercars.car{i}.flgPlaza == 0
        othercars.car{i}.pos ...
            = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T);
        othercars.car{i}.bd ...
            = get_carshape(othercars.car{i}.pos ...
            , othercars.car{i}.W, othercars.car{i}.H);
        if othercars.car{i}.pos(1) > 100*10^3 && othercars.car{i}.pos(1) < 275*10^3
            othercars.car{i}.flgPlaza = 1;
            
            dx = 175*10^3/3; % x-cood.interval of control points for bezier curve
            ctlPt = [0 0; dx 0; 2*dx 77.5*10^3 - othercars.car{i}.pos(2) - 5*10^3*othercars.car{i}.tolllane; 3*dx 77.5*10^3 - othercars.car{i}.pos(2) - 5*10^3*othercars.car{i}.tolllane];
            [laneChangePath, lengthP] = bezierCurve(ctlPt);
            ratioSpeed = lengthP/(175*10^3)*1.1;
            %othercars.car{i}.vel(1) = othercars.car{i}.vel(1)*ratioSpeed;
            othercars.car{i}.pathTranslated = update_laneChangePath(othercars.car{i},laneChangePath);
            
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