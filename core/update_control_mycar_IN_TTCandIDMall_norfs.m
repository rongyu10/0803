function [mycar, othercars] = update_control_mycar_IN_TTCandIDMall_norfs(mycar, sim, othercars, idm, laneChangePath)

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
time_TTC = 5.0;
step_TTC = 0.1;

idx_crashcar = [];


if mycar.flgPlaza == 0 % before entering plaza
    
    v0 = 15000;
    mycar.acceleration = 0;
    
    if mycar.pos(1) > 0
        [idx_crashcar, t, mycar_posEst, idx_frontCar] = is_carcrashed_formycar_TTC_verIDM_inpol4_mycar(othercars, time_TTC, step_TTC, mycar);
        
        if ~isempty(idx_crashcar)
            
            fprintf(1, 'after [%d] seconds, mycar and [%d] collide\n', t, idx_crashcar);
            
            A3 = norm(mycar_posEst(1:2) - othercars.car{idx_crashcar}.pos(1:2));
            A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{idx_crashcar}.vel(1))/2/sqrt(a*b))/A3;
            A1 = mycar.vel(1)/v0;
            mycar.acceleration = a*(1 - A1^delta - A2^2);
            
        else
            c = cos(mycar.pos(3)*pi/180);
            s = sin(mycar.pos(3)*pi/180);
            squareX = [mycar.pos(1)+2500*s mycar.pos(1)+2500*s+30000*c mycar.pos(1)-2500*s+30000*c mycar.pos(1)-2500*s mycar.pos(1)+2500*s];
            squareY = [mycar.pos(2)-2500*c mycar.pos(2)-2500*c+30000*s mycar.pos(2)+2500*c+30000*s mycar.pos(2)+2500*c mycar.pos(2)-2500*c];
            
            nr_frontCar = length(idx_frontCar);
            idx_mindist = [];
            dist_min = 30000;
            for i = 1:nr_frontCar
                in = inpolygon(othercars.car{idx_frontCar(i)}.pos(1), othercars.car{idx_frontCar(i)}.pos(2), squareX, squareY);
                if in == 1
                    if norm(mycar.pos(1:2) - othercars.car{idx_frontCar(i)}.pos(1:2)) < dist_min
                        dist_min = norm(mycar.pos(1:2) - othercars.car{idx_frontCar(i)}.pos(1:2));
                        idx_mindist = idx_frontCar(i);
                    end
                end
            end
            
            if ~isempty(idx_mindist) % IDM following frontcar
                fprintf(1, 'Following car [%d] by IDM\n',idx_mindist);
                A3 = norm(othercars.car{idx_mindist}.pos(1:2) - mycar.pos(1:2)) - l;
                A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{idx_mindist}.vel(1))/2/sqrt(a*b))/A3;
            else
                A2 = 0;
            end
            A1 = mycar.vel(1)/v0;
            mycar.acceleration = a*(1 - A1^delta - A2^2);
        end
        
        if mycar.acceleration < -9800
            fprintf(2, 'mycar acceleration = [%4d] \n', mycar.acceleration);
            mycar.acceleration = -9800;
        else
            fprintf(1, 'mycar acceleration = [%4d] \n', mycar.acceleration);
        end
        
    end
    
    % if entering the plaza
    if mycar.pos(1) > 100*10^3 && mycar.pos(1) < 275*10^3
        mycar.flgPlaza = 1;
        
        mycar.pathTranslated = laneChangePath{mycar.selectlane, mycar.save.lane_idx};
    end
    
elseif mycar.flgPlaza == 1 % after entering plaza
    
    pos = predict_pos(mycar.pos, mycar.vel, sim.T);
    targetDegree = get_tatgetTheta(pos,mycar.pathTranslated);
    mycar.pos(3) = targetDegree;
    
    if mycar.pos(1) < 187.5*10^3
        v0 = 12500;
        [idx_crashcar, t, mycar_posEst, idx_frontCar] = is_carcrashed_formycar_TTC_verIDM_inpol4_mycar(othercars, time_TTC, step_TTC, mycar);
        
        if ~isempty(idx_crashcar)
            
            fprintf(1, 'after [%d] seconds, mycar and [%d] collide\n', t, idx_crashcar);
            
            A3 = norm(mycar_posEst(1:2) - othercars.car{idx_crashcar}.pos(1:2));
            A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{idx_crashcar}.vel(1))/2/sqrt(a*b))/A3;
            A1 = mycar.vel(1)/v0;
            mycar.acceleration = a*(1 - A1^delta - A2^2);
            
        else
            c = cos(mycar.pos(3)*pi/180);
            s = sin(mycar.pos(3)*pi/180);
            squareX = [mycar.pos(1)+2500*s mycar.pos(1)+2500*s+30000*c mycar.pos(1)-2500*s+30000*c mycar.pos(1)-2500*s mycar.pos(1)+2500*s];
            squareY = [mycar.pos(2)-2500*c mycar.pos(2)-2500*c+30000*s mycar.pos(2)+2500*c+30000*s mycar.pos(2)+2500*c mycar.pos(2)-2500*c];
            
            nr_frontCar = length(idx_frontCar);
            idx_mindist = [];
            dist_min = 30000;
            for i = 1:nr_frontCar
                in = inpolygon(othercars.car{idx_frontCar(i)}.pos(1), othercars.car{idx_frontCar(i)}.pos(2), squareX, squareY);
                if in == 1
                    if norm(mycar.pos(1:2) - othercars.car{idx_frontCar(i)}.pos(1:2)) < dist_min
                        dist_min = norm(mycar.pos(1:2) - othercars.car{idx_frontCar(i)}.pos(1:2));
                        idx_mindist = idx_frontCar(i);
                    end
                end
            end
            
            if ~isempty(idx_mindist) % IDM following frontcar
                fprintf(1, 'Following car [%d] by IDM\n',idx_mindist);
                A3 = norm(othercars.car{idx_mindist}.pos(1:2) - mycar.pos(1:2)) - l;
                A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{idx_mindist}.vel(1))/2/sqrt(a*b))/A3;
            else
                A2 = 0;
            end
            A1 = mycar.vel(1)/v0;
            mycar.acceleration = a*(1 - A1^delta - A2^2);
        end
        
        if mycar.acceleration < -9800
            fprintf(2, 'mycar acceleration = [%4d] \n', mycar.acceleration);
            mycar.acceleration = -9800;
        else
            fprintf(1, 'mycar acceleration = [%4d] \n', mycar.acceleration);
        end
        
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
        
        [idx_crashcar, t, mycar_posEst, idx_frontCar] = is_carcrashed_formycar_TTC_verIDM_inpol4_mycar(othercars, time_TTC, step_TTC, mycar);
        
        if ~isempty(idx_crashcar)
            
            fprintf(1, 'after [%d] seconds, mycar and [%d] collide\n', t, idx_crashcar);
            
            A3 = norm(mycar_posEst(1:2) - othercars.car{idx_crashcar}.pos(1:2));
            A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{idx_crashcar}.vel(1))/2/sqrt(a*b))/A3;
            A1 = mycar.vel(1)/v0;
            mycar.acceleration = a*(1 - A1^delta - A2^2);
            
        else
            c = cos(mycar.pos(3)*pi/180);
            s = sin(mycar.pos(3)*pi/180);
            squareX = [mycar.pos(1)+2500*s mycar.pos(1)+2500*s+30000*c mycar.pos(1)-2500*s+30000*c mycar.pos(1)-2500*s mycar.pos(1)+2500*s];
            squareY = [mycar.pos(2)-2500*c mycar.pos(2)-2500*c+30000*s mycar.pos(2)+2500*c+30000*s mycar.pos(2)+2500*c mycar.pos(2)-2500*c];
            
            nr_frontCar = length(idx_frontCar);
            idx_mindist = [];
            dist_min = 30000;
            for i = 1:nr_frontCar
                in = inpolygon(othercars.car{idx_frontCar(i)}.pos(1), othercars.car{idx_frontCar(i)}.pos(2), squareX, squareY);
                if in == 1
                    if norm(mycar.pos(1:2) - othercars.car{idx_frontCar(i)}.pos(1:2)) < dist_min
                        dist_min = norm(mycar.pos(1:2) - othercars.car{idx_frontCar(i)}.pos(1:2));
                        idx_mindist = idx_frontCar(i);
                    end
                end
            end
            
            if ~isempty(idx_mindist) % IDM following frontcar
                fprintf(1, 'Following car [%d] by IDM\n',idx_mindist);
                A3 = norm(othercars.car{idx_mindist}.pos(1:2) - mycar.pos(1:2)) - l;
                A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{idx_mindist}.vel(1))/2/sqrt(a*b))/A3;
            else
                A2 = 0;
            end
            A1 = mycar.vel(1)/v0;
            mycar.acceleration = a*(1 - A1^delta - A2^2);
            
        end
        
        if mycar.acceleration < -9800
            fprintf(2, 'mycar acceleration = [%4d] \n', mycar.acceleration);
            mycar.acceleration = -9800;
        else
            fprintf(1, 'mycar acceleration = [%4d] \n', mycar.acceleration);
        end
        
    elseif mycar.pos(1) < 320*10^3
        
        [~, ~, ~, idx_frontCar] = is_carcrashed_formycar_TTC_verIDM_inpol4_mycar(othercars, time_TTC, step_TTC, mycar);
        
        c = cos(mycar.pos(3)*pi/180);
        s = sin(mycar.pos(3)*pi/180);
        squareX = [mycar.pos(1)+2500*s mycar.pos(1)+2500*s+30000*c mycar.pos(1)-2500*s+30000*c mycar.pos(1)-2500*s mycar.pos(1)+2500*s];
        squareY = [mycar.pos(2)-2500*c mycar.pos(2)-2500*c+30000*s mycar.pos(2)+2500*c+30000*s mycar.pos(2)+2500*c mycar.pos(2)-2500*c];
        
        nr_frontCar = length(idx_frontCar);
        idx_mindist = [];
        dist_min = 30000;
        for i = 1:nr_frontCar
            in = inpolygon(othercars.car{idx_frontCar(i)}.pos(1), othercars.car{idx_frontCar(i)}.pos(2), squareX, squareY);
            if in == 1
                if norm(mycar.pos(1:2) - othercars.car{idx_frontCar(i)}.pos(1:2)) < dist_min
                    dist_min = norm(mycar.pos(1:2) - othercars.car{idx_frontCar(i)}.pos(1:2));
                    idx_mindist = idx_frontCar(i);
                end
            end
        end
        
        if ~isempty(idx_mindist) % IDM following frontcar
            fprintf(1, 'Following car [%d] by IDM\n',idx_mindist);
            A3 = norm(othercars.car{idx_mindist}.pos(1:2) - mycar.pos(1:2)) - l;
            A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{idx_mindist}.vel(1))/2/sqrt(a*b))/A3;
        else
            A2 = 0;
        end
        A1 = mycar.vel(1)/v0;
        mycar.acceleration = a*(1 - A1^delta - A2^2);
        
        if mycar.acceleration < -9800
            fprintf(2, 'mycar acceleration = [%4d] \n', mycar.acceleration);
            mycar.acceleration = -9800;
        else
            fprintf(1, 'mycar acceleration = [%4d] \n', mycar.acceleration);
        end
    end
end

mycar.vel(1) = mycar.vel(1) + mycar.acceleration*sim.T;

% decelerate in the toll lane
if mycar.pos(1) > 275*10^3 && mycar.vel(1) > 5000
    mycar.vel(1) = mycar.vel(1) - 3000 * sim.T; % decelerate if exceeds 18km/h in the tolllane
end

% control minimum velocity
if mycar.vel(1) < 0 && mycar.pos(1) < 320 * 10^3
    mycar.vel(1) = 0;
end

% UPDATE MY CAR INFORMATION
mycar.pos = update_pos(mycar.pos, mycar.vel, sim.T);
mycar.bd  = get_carshape(mycar.pos, mycar.W, mycar.H);

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