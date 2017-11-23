function othercars = calculate_velocity_othercars_tollPlaza_IN(othercars, sim, mycar, idm, laneChangePath)

for i = 1:othercars.n
    
    if othercars.car{i}.flgPlaza == 0 % before entering plaza
        idm.v0 = 15000;
         
    elseif othercars.car{i}.flgPlaza == 1 % after entering plaza
        pos = predict_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T);
        othercars.car{i}.targetDegree = get_tatgetTheta(pos,othercars.car{i}.pathTranslated);
        if othercars.car{i}.targetDegree - othercars.car{i}.pos(3) > 10
            othercars.car{i}.vel(2) = othercars.car{i}.vel(2) + 10/sim.T;
        elseif othercars.car{i}.targetDegree - othercars.car{i}.pos(3) < -10
            othercars.car{i}.vel(2) = othercars.car{i}.vel(2) - 10/sim.T;
        else
            othercars.car{i}.vel(2) = othercars.car{i}.vel(2) + (othercars.car{i}.targetDegree - othercars.car{i}.pos(3))/sim.T;
        end
        
        if othercars.car{i}.pos(1) < 187.5*10^3
            idm.v0 = 15000;
            
        elseif othercars.car{i}.pos(1) < 275*10^3
            idm.v0 = 12500;
            
        elseif othercars.car{i}.pos(1) < 320*10^3
            idm.v0 = 10000;
        end
        
    end
    
    [idx_observedcar, t_observedcar, pos_mycarEst, pos_observedcarEst] = is_carcrashed_orFollow_forothercars_TTC_forwardrect(othercars, i, mycar, laneChangePath);    
        
    if ~isempty(idx_observedcar)
        
        nr_observedCar = length(idx_observedcar);
        idx_maxDecelerate = [];
        t_maxDecelerate = [];
        min_acceleration = 0;
        
        
        % iterate by number of estimated collision cars (to calculate deceleration by TTC)
        for j = 1:nr_observedCar
            
%             if abs(pos_mycarEst(3) - pos_observedcarEst(3)) < 5
%                 FLAG_Follow_or_Cross = 1;
%             else
%                 FLAG_Follow_or_Cross = 0;
%             end

            rel_degree = abs(pos_mycarEst(j,3) - pos_observedcarEst(j,3));
            
            if idx_observedcar(j) == 0
                cur_acceleration = calculate_acceleration_IDM(othercars.car{i}, mycar, pos_observedcarEst(j,:), idm, rel_degree);
            else
                cur_acceleration = calculate_acceleration_IDM(othercars.car{i}, othercars.car{idx_observedcar(j)}, pos_observedcarEst(j,:), idm, rel_degree);
            end
            
            
            
            %fprintf(1, 'after [%d] seconds, mycar and [%d](%d, %d) collide at (%d, %d)\n', t, idx_observedcar, othercars.car{idx_observedcar(i)}.pos(1), othercars.car{idx_observedcar(i)}.pos(2), pos_mycarEst(1), pos_mycarEst(2));
            
            %A3 = norm(othercars.car{idx_observedcar(i)}.pos(1:2) - mycar.pos(1:2));
%             othercar_posEst_i = pos_observedcarEst(j,:);
%             A3_TTC = norm(othercar_posEst_i(1:2) - othercars.car{i}.pos(1:2)) - l;
%             if A3_TTC < s0
%                 A3_TTC = s0;
%             end
%             
%             if FLAG_Follow_or_Cross
%                 if idx_observedcar(j) == 0 % for mycar
%                     A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - (mycar.vel(1)*cos((mycar.pos(3)-othercars.car{i}.pos(3))*pi/180)))/2/sqrt(a*b))/A3_TTC;
%                 else % for othercar
%                     A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - (othercars.car{idx_observedcar(j)}.vel(1)*cos((othercars.car{idx_observedcar(j)}.pos(3)-othercars.car{i}.pos(3))*pi/180)))/2/sqrt(a*b))/A3_TTC;
%                 end
%             else
%                 A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * othercars.car{i}.vel(1)/2/sqrt(a*b))/A3_TTC;
%             end
%             
%             A1 = othercars.car{i}.vel(1)/idm.v0;
%             accele_IDM = a*(1 - A1^delta - A2^2);
%             
%             % -----ACC model-----------
%             aLead = 0;  % this value need to be modified !!
%             aLeadRestricted = min(aLead,a);
%             
%             if FLAG_Follow_or_Cross
%                 if idx_observedcar(j) == 0 % for mycar
%                     dvp = max(othercars.car{i}.vel(1) - mycar.vel(1)*cos((mycar.pos(3)-othercars.car{i}.pos(3))*pi/180),0);
%                     vLead = mycar.vel(1)*cos((mycar.pos(3)-othercars.car{i}.pos(3))*pi/180);
%                 else % for othercar
%                     dvp = max(othercars.car{i}.vel(1) - othercars.car{idx_observedcar(j)}.vel(1)*cos((othercars.car{idx_observedcar(j)}.pos(3)-othercars.car{i}.pos(3))*pi/180),0);
%                     vLead = othercars.car{idx_observedcar(j)}.vel(1);
%                 end
%             else
%                 dvp = max(othercars.car{i}.vel(1),0);
%                 vLead = 0;
%             end
%             
%             denomCAH = vLead*vLead - 2*A3_TTC*aLeadRestricted;
%             
%             if (vLead*dvp < -2*A3_TTC*aLeadRestricted)&&(denomCAH~=0)
%                 accele_CAH = othercars.car{i}.vel(1)*othercars.car{i}.vel(1)*aLeadRestricted/denomCAH;
%             else
%                 accele_CAH = aLeadRestricted - 0.5*dvp*dvp/max(A3_TTC,0.1);
%             end
%             
%             if accele_IDM > accele_CAH
%                 cur_acceleration = accele_IDM;
%             else
%                 cur_acceleration = (1-coolness)*accele_IDM + coolness*( accele_CAH + b*tanh((accele_IDM - accele_CAH)/b));
%             end
            
            % -----end of ACC model---------------
            
            if i == 1
                min_acceleration = cur_acceleration;
            end
            
            if cur_acceleration <= min_acceleration
                othercars.car{i}.acceleration = cur_acceleration;
                idx_maxDecelerate = idx_observedcar(j);
                t_maxDecelerate = t_observedcar(j);
            end
        end
        
        
%         if idx_maxDecelerate == 0
%             if FLAG_Follow_or_Cross
%                 if othercars.car{i}.acceleration < -2940
%                     fprintf(2, 'car[%d]([%d], [%d]) decelerate([%d]) to mycar (distance = [%d], observed time = [%d], reldegree = [%d]) by FOLLOW. Observed car position is [%d, %d]. A1=[%d], A2=[%d], A3_TTC=[%d]\n', i, othercars.car{i}.pos(1), othercars.car{i}.pos(2), othercars.car{i}.acceleration, A3_TTC, t_maxDecelerate, pos_mycarEst(3) - pos_observedcarEst(3), othercar_posEst_i(1), othercar_posEst_i(2), A1, A2, A3_TTC);
%                 else
%                     fprintf(1, 'car[%d]([%d], [%d]) decelerate([%d]) to mycar (distance = [%d], observed time = [%d], reldegree = [%d]) by FOLLOW. Observed car position is [%d, %d]. A1=[%d], A2=[%d], A3_TTC=[%d]\n', i, othercars.car{i}.pos(1), othercars.car{i}.pos(2), othercars.car{i}.acceleration, A3_TTC, t_maxDecelerate, pos_mycarEst(3) - pos_observedcarEst(3), othercar_posEst_i(1), othercar_posEst_i(2), A1, A2, A3_TTC);
%                 end
%             else
%                 if othercars.car{i}.acceleration < -2940
%                     fprintf(2, 'car[%d]([%d], [%d]) decelerate([%d]) to mycar (distance = [%d], observed time = [%d], reldegree = [%d]) by CROSS. Observed car position is [%d, %d]. A1=[%d], A2=[%d], A3_TTC=[%d]\n', i, othercars.car{i}.pos(1), othercars.car{i}.pos(2), othercars.car{i}.acceleration, A3_TTC, t_maxDecelerate, pos_mycarEst(3) - pos_observedcarEst(3), othercar_posEst_i(1), othercar_posEst_i(2), A1, A2, A3_TTC);
%                 else
%                     fprintf(1, 'car[%d]([%d], [%d]) decelerate([%d]) to mycar (distance = [%d], observed time = [%d], reldegree = [%d]) by CROSS. Observed car position is [%d, %d]. A1=[%d], A2=[%d], A3_TTC=[%d]\n', i, othercars.car{i}.pos(1), othercars.car{i}.pos(2), othercars.car{i}.acceleration, A3_TTC, t_maxDecelerate, pos_mycarEst(3) - pos_observedcarEst(3), othercar_posEst_i(1), othercar_posEst_i(2), A1, A2, A3_TTC);
%                 end
%             end
% %             if FLAG_Follow_or_Cross
% %                 fprintf(1, 'car[%d] decelerate to car [%d] (distance = [%d], observed time = [%d], reldegree = [%d]) by FOLLOW. Observed car position is [%d, %d]. A1=[%d], A2=[%d], A3_TTC=[%d]\n', i, idx_maxDecelerate, A3_TTC, t_maxDecelerate, pos_mycarEst(3) - pos_observedcarEst(3), othercar_posEst_i(1), othercar_posEst_i(2), A1, A2, A3_TTC);
% %             else
% %                 fprintf(1, 'car[%d] decelerate to car [%d] (distance = [%d], observed time = [%d], reldegree = [%d]) by CROSS. Observed car position is [%d, %d]. A1=[%d], A2=[%d], A3_TTC=[%d]\n', i, idx_maxDecelerate, A3_TTC, t_maxDecelerate, pos_mycarEst(3) - pos_observedcarEst(3), othercar_posEst_i(1), othercar_posEst_i(2), A1, A2, A3_TTC);
% %             end
%         end
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
    
% update position after updating all the velocity of othercars
% for i = 1:othercars.n
%     othercars.car{i}.vel(1) = othercars.car{i}.vel(1) + othercars.car{i}.acceleration*sim.T;
%     
%     % control minimum velocity
%     if othercars.car{i}.vel(1) < 0 && othercars.car{i}.pos(1) < 320 * 10^3
%         othercars.car{i}.vel(1) = 0;
%     end
%     
%     % UPDATE MY CAR INFORMATION
%     othercars.car{i}.pos = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T);
%     othercars.car{i}.bd  = get_carshape(othercars.car{i}.pos, othercars.car{i}.W, othercars.car{i}.H);
%     
%     % if entering the plaza
%     if othercars.car{i}.pos(1) > 100*10^3 && othercars.car{i}.pos(1) < 275*10^3
%         othercars.car{i}.flgPlaza = 1;
%         
%         othercars.car{i}.pathTranslated = laneChangePath{othercars.car{i}.goallane, othercars.car{i}.save.lane_idx};
%     end
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