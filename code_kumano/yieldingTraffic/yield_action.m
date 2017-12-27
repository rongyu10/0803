function othercars = yield_action(mycar, othercars, sim)
%
% 

ACCEPT_VEL  = 2000; % Acceptable relative velocity [mm/s]

[~,idx_rear] = get_neighboursCars(mycar, othercars);

for i = 1:othercars.n
    othercars.car{i}.yield = 0;
end

if idx_rear ==0
    return
end

if isfield(mycar,'turnSignal')==0
    return
elseif strcmp(mycar.turnSignal,'right')&& (othercars.car{idx_rear}.IDM.polite ==1)
    mycar_pos     = mycar.pos(1);
    othercar_pos  = othercars.car{idx_rear}.pos(1);
    [car_size, ~] = get_carsize();
    
     % ( abs(relative_velocity) < threshold )&&( x-coord diff > threshold) 
    if (abs(mycar.vel(1) - othercars.car{idx_rear}.vel(1))<ACCEPT_VEL)&&((mycar_pos - othercar_pos) > car_size)
        othercars.car{idx_rear}.yield = 1;
 
        v_free = othercars.car{idx_rear}.IDM.v0;
        % driver model---
        %[accele, angry] = predictAccel_by_IDM(v_free, mycar, othercars.car{idx_rear}, false);
        [accele, angry]  = predictAccel_by_ACC(v_free, mycar, othercars.car{idx_rear}, false);
        %----------------
        v = othercars.car{idx_rear}.vel(1);
        RATIO_DECEL = othercars.car{idx_rear}.IDM.ratio_decel;
        if accele <  0
            if angry > 0
               accele = -3000;
            end
            accele = accele*RATIO_DECEL;
        end
        
        v_new = v + accele * sim.T;
        othercars.car{idx_rear}.vel(1) = max(0.0,v_new); % 速度は正 
        othercars.car{idx_rear}.accele = accele;
    end
    
else
    return
end

end
