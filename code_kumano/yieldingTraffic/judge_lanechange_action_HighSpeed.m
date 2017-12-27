function [flag_LC, data_LC] = judge_lanechange_action_HighSpeed(mycar, myinfo, othercars, sim, data_LC, flag_congested)
%
% 

ndata = size(data_LC,1);


%- Setting -------------------------------
if flag_congested
   ratio_carLength = 2; 
else
   ratio_carLength = 2.5;     
end

accept_accel = -2000;
%------------------------------------------


%--- Saving Lane Change Judgment Data------
idx = find(data_LC==-10,1);
if ~isempty(idx)
    data_LC(idx,1)= mycar.lanechange;
else
    tmp_data = zeros(ndata,1);
    tmp_data(1:end-1,1) = data_LC(2:end,1);
    tmp_data(end,1) = mycar.lanechange;
    data_LC  = tmp_data;
end
%------------------------------------------

mean_data_LC = mean(data_LC);

if mean_data_LC > 0.8
   flag_LC = 1;
else
   flag_LC = 0; 
end

%-------------------------------------------------
[idx_front,idx_rear] = get_neighboursCars(mycar, othercars);

if (idx_front ~=0)&&(idx_rear ~=0)
    v_free = othercars.car{idx_rear}.IDM.v0;
    [accele_by_mycar, ~]  = predictAccel_by_ACC(v_free, mycar, othercars.car{idx_rear}, true);
    diff_pos1 = othercars.car{idx_front}.pos(1)-mycar.pos(1);
    diff_pos2 = mycar.pos(1) - othercars.car{idx_rear}.pos(1);
    
    if (diff_pos1 > (mycar.W)*ratio_carLength)&&(diff_pos2 > (mycar.W)*ratio_carLength)&&(accele_by_mycar> accept_accel)
       flag_LC = 1;
       return
    else
       flag_LC = 0;
       return        
    end
    
elseif (idx_front ==0)&&(idx_rear ~=0)   % in case of no leading car 
    v_free = othercars.car{idx_rear}.IDM.v0;
    [accele_by_mycar, ~]  = predictAccel_by_ACC(v_free, mycar, othercars.car{idx_rear}, true);
    if (accele_by_mycar> accept_accel)
       flag_LC = 1;
       return
    else
       flag_LC = 0;
       return                
    end
    
elseif(idx_front ~=0)&&(idx_rear ==0)   % in case of no following car
    diff_pos = othercars.car{idx_front}.pos(1)-mycar.pos(1);
    if (diff_pos > (mycar.W)*ratio_carLength)
       flag_LC = 1;
       return
    else
       flag_LC = 0;
       return                
    end
end
%----------------------------------------------------


end


