function mycar = judge_yield_action(mycar,myinfo, othercars, sim)
%
% 

[idx_front,idx_rear] = get_neighboursCars(mycar, othercars);

if (idx_front ~=0)&&(idx_rear ~=0)
    v_free = othercars.car{idx_rear}.IDM.v0;
    [accele_by_frontcar, ~]  = predictAccel_by_ACC(v_free, othercars.car{idx_front}, othercars.car{idx_rear}, true);
    [accele_by_mycar, ~]  = predictAccel_by_ACC(v_free, mycar, othercars.car{idx_rear}, true);
    actual_accele  = othercars.car{idx_rear}.accele;
elseif (idx_front ==0)&&(idx_rear ~=0)   % in case of no leading car 
    v_free = othercars.car{idx_rear}.IDM.v0;
    [accele_by_frontcar, ~]  = predictAccel_by_ACC(v_free, [], othercars.car{idx_rear}, true);
    [accele_by_mycar, ~]  = predictAccel_by_ACC(v_free, mycar, othercars.car{idx_rear}, true);
    actual_accele  = othercars.car{idx_rear}.accele;    
else
    accele_by_frontcar = NaN;
    accele_by_mycar    = NaN;
    actual_accele      = NaN;
end

[prob_front, prob_mycar] = evaluate_probability(actual_accele, accele_by_frontcar, accele_by_mycar);

if isnumeric(prob_front)&&isnumeric(prob_mycar)&&(myinfo.lane_idx==1)
    if prob_mycar > (prob_front + 0.1)
        mycar.lanechange = 1;
    else
        mycar.lanechange = 0;
    end
else
        mycar.lanechange = 0;    
end

%fprintf('actual:%.1f,  accle by the front: %.1f,  accle by mycar: %.1f \n', actual_accele, accele_by_frontcar, accele_by_mycar);
%fprintf('prob_front: %.1f %%,  prob_mycar: %.1f %% \n', prob_front*100, prob_mycar*100);
%fprintf('LaneChange OK: %.1f %% ,   NG: %.1f %% \n', prob_mycar*100, prob_front*100);

end

function [prob_front, prob_mycar] = evaluate_probability(actual_accele, accele_by_frontcar, accele_by_mycar)
% evaluate probability by linear function

if isempty(actual_accele)
    prob_front = NaN;
    prob_mycar = NaN;
    return

elseif (isnan(actual_accele))||(isnan(accele_by_frontcar))||(isnan(accele_by_mycar))
    prob_front = NaN;
    prob_mycar = NaN;
    return
elseif accele_by_frontcar <  accele_by_mycar
    prob_front = NaN;
    prob_mycar = NaN;
    return
    
elseif actual_accele > accele_by_frontcar
    prob_front = 1.0;
    prob_mycar = 0.0;
    return
    
elseif actual_accele < accele_by_mycar
    prob_front = 0.0;
    prob_mycar = 1.0;
    return
    
else
    interval = accele_by_frontcar - accele_by_mycar;
    prob_front = (actual_accele - accele_by_mycar)/interval;
    prob_mycar = (accele_by_frontcar - actual_accele)/interval;    

end


end
