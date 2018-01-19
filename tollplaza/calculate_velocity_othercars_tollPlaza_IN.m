function othercars = calculate_velocity_othercars_tollPlaza_IN(othercars, sim, mycar, idm, laneChangePath)

for i = 1:othercars.n
    
    if othercars.car{i}.flgPlaza == 0 % before entering plaza
        idm.v0 = 17500;
        % if entering the plaza
        if othercars.car{i}.pos(1) > 100*10^3 && othercars.car{i}.pos(1) < 275*10^3
            othercars.car{i}.flgPlaza = 1;
        end
         
    elseif othercars.car{i}.flgPlaza == 1 % after entering plaza
        pos = predict_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T);
        targetDegree = get_tatgetTheta(pos,othercars.car{i}.pathTranslated);
        othercars.car{i}.vel(2) = (targetDegree - othercars.car{i}.pos(3))/sim.T;
%         if othercars.car{i}.targetDegree - othercars.car{i}.pos(3) > 10
%             othercars.car{i}.vel(2) = othercars.car{i}.vel(2) + 10/sim.T;
%         elseif othercars.car{i}.targetDegree - othercars.car{i}.pos(3) < -10
%             othercars.car{i}.vel(2) = othercars.car{i}.vel(2) - 10/sim.T;
%         else
%             othercars.car{i}.vel(2) = othercars.car{i}.vel(2) + (othercars.car{i}.targetDegree - othercars.car{i}.pos(3))/sim.T;
%         end
        
        % 目標速度をゲートからの距離に応じて27km/h〜63km/hに設定
        idm.v0 = 7500 + 10000*(320*10^3 - othercars.car{i}.pos(1))/(220*10^3);
        
    end
    
    [idx_observedcar, t_observedcar, pos_mycarEst, observedcarEst, othercars] = detect_frontcar_forothercars(othercars, i, mycar, laneChangePath);    
    
    if ~isempty(idx_observedcar)
        
        nr_observedCar = length(idx_observedcar);
        idx_maxDecelerate = [];
        t_maxDecelerate = [];
        min_acceleration = 0;
        
        % iterate by number of estimated collision cars (to calculate deceleration by TTC)
        for j = 1:nr_observedCar

            rel_degree = abs(pos_mycarEst(j,3) - observedcarEst(j).pos(3));
            
            cur_acceleration = calculate_acceleration_IDM(othercars.car{i}, observedcarEst(j), idm, rel_degree);
            
            %fprintf(1, 'after [%d] seconds, mycar and [%d](%d, %d) collide at (%d, %d)\n', t, idx_observedcar, othercars.car{idx_observedcar(i)}.pos(1), othercars.car{idx_observedcar(i)}.pos(2), pos_mycarEst(1), pos_mycarEst(2));
            
            if j == 1
                min_acceleration = cur_acceleration;
            end
            
            if cur_acceleration <= min_acceleration
                othercars.car{i}.acceleration = cur_acceleration;
                idx_maxDecelerate = idx_observedcar(j);
                t_maxDecelerate = t_observedcar(j);
                deg_maxDecelerate = rel_degree;
            end
        end
        
        % --regulate acceleration and velocity ---------------------
%         if othercars.car{i}.acceleration > othercars.max_acceleration
%             othercars.car{i}.acceleration = othercars.max_acceleration;
%         elseif othercars.car{i}.acceleration < -othercars.max_acceleration
%             othercars.car{i}.acceleration = -othercars.max_acceleration;
%         end
        
%         if idx_maxDecelerate == 0
%             if othercars.car{i}.acceleration < -othercars.max_acceleration
%                 fprintf(2, 'car[%d]([%d], [%d]) decelerate([%d]) to mycar (observed time = [%d], reldegree = [%d]). Mycar position is [%d, %d].\n', i, othercars.car{i}.pos(1), othercars.car{i}.pos(2), othercars.car{i}.acceleration, t_maxDecelerate, deg_maxDecelerate, mycar.pos(1), mycar.pos(2));
%             else
%                 fprintf(1, 'car[%d]([%d], [%d]) decelerate([%d]) to mycar (observed time = [%d], reldegree = [%d]). Mycar position is [%d, %d].\n', i, othercars.car{i}.pos(1), othercars.car{i}.pos(2), othercars.car{i}.acceleration, t_maxDecelerate, deg_maxDecelerate, mycar.pos(1), mycar.pos(2));
%             end
%         elseif idx_maxDecelerate ~= 0
%             if othercars.car{i}.acceleration < -othercars.max_acceleration
%                 fprintf(2, 'car[%d]([%d], [%d]) decelerate([%d]) to car[%d] (observed time = [%d], reldegree = [%d]). Mycar position is [%d, %d].\n', i, othercars.car{i}.pos(1), othercars.car{i}.pos(2), othercars.car{i}.acceleration, idx_maxDecelerate, t_maxDecelerate, deg_maxDecelerate, mycar.pos(1), mycar.pos(2));
%             else
%                 fprintf(1, 'car[%d]([%d], [%d]) decelerate([%d]) to car[%d] (observed time = [%d], reldegree = [%d]). Mycar position is [%d, %d].\n', i, othercars.car{i}.pos(1), othercars.car{i}.pos(2), othercars.car{i}.acceleration, idx_maxDecelerate, t_maxDecelerate, deg_maxDecelerate, mycar.pos(1), mycar.pos(2));
%             end
%         end
        
        if idx_maxDecelerate == 0
            if othercars.car{i}.acceleration < -othercars.max_acceleration
                fprintf(2, 'car[%d](%6.0f, %4.0f) decelerate([%4.0f]) to mycar (observed time = [%1.2f], reldegree = [%2.1f]). Mycar position is [%6.0f, %4.0f].\n', i, othercars.car{i}.pos(1), othercars.car{i}.pos(2), othercars.car{i}.acceleration, t_maxDecelerate, deg_maxDecelerate, mycar.pos(1), mycar.pos(2));
            end
        elseif idx_maxDecelerate ~= 0
%             if othercars.car{i}.acceleration < -othercars.max_acceleration
%                 fprintf(2, 'car[%d](%6.0f, %4.0f) decelerate([%4.0f]) to car[%d] (observed time = [%1.2f], reldegree = [%2.1f]). Mycar position is [%6.0f, %4.0f].\n', i, othercars.car{i}.pos(1), othercars.car{i}.pos(2), othercars.car{i}.acceleration, idx_maxDecelerate, t_maxDecelerate, deg_maxDecelerate, mycar.pos(1), mycar.pos(2));
%             end
        end

    else
        % if there is no car in detecting area
        A1 = othercars.car{i}.vel(1)/idm.v0;
        othercars.car{i}.acceleration = idm.a*(1 - A1^idm.delta);
    end
    
    
    othercars.car{i}.vel(1) = othercars.car{i}.vel(1) + othercars.car{i}.acceleration*sim.T;
    
    % control minimum velocity
    if othercars.car{i}.vel(1) < 0 && othercars.car{i}.pos(1) < 320 * 10^3
        othercars.car{i}.vel(1) = 0;
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