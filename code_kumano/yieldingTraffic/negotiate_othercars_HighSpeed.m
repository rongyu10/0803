function mycar = negotiate_othercars_HighSpeed(mycar, othercars, sim, flag_congested)
% Chase target
persistent first_flag idx_front idx_rear
if isempty(first_flag)
    first_flag = true;
end

%--- turn signal -------
if sim.tick > 2
    mycar.turnSignal =  'right';
end
%-----------------------

%-----Setting for Negotiation -----------------
ratio_carLength = 3;
%----------------------------------------------

%--- Forcing Merging (Urgent Brake)-------
if mycar.pos(1)>180*10^3
   mycar.vel(1) = 0;
   mycar.vel(2) = -3;
   if mycar.pos(3)<-10
       mycar.vel(2) = 0;
   end
   return
end
%-----------------------------------------


%--- Moving to the first target----------------------------------
    [~,idx_front1,idx_rear1,~] = get_neighboursCars2(mycar, othercars);
    idx_front = idx_front1;
    idx_rear  = idx_rear1;
    %---- no other cars in front/rear of mycar---
    if (idx_front ==0)&&(idx_rear ==0)
       return
    end
    %--------------------------------------------
    if (idx_front ==0)
       pos           = mycar.pos(1);
       pos_rearcar  = othercars.car{idx_rear}.pos(1);
       err = pos - pos_rearcar;
       if err < (mycar.W)*ratio_carLength
          mycar.vel(1) = mycar.vel(1) - 1000*sim.T;
          return
       else
          %mycar.vel(1) = mycar.vel(1) + 1000*sim.T;
          return
       end
    end
    %---------------------------------------------
    if (idx_rear ==0)
       pos           = mycar.pos(1);
       pos_frontcar  = othercars.car{idx_front}.pos(1);
       err = pos_frontcar - pos;
       if err < (mycar.W)*ratio_carLength
          mycar.vel(1) = mycar.vel(1) - 1000*sim.T;
          return
       else
          %mycar.vel(1) = mycar.vel(1) - 500*sim.T;
          return
       end
    end
    %---------------------------------------------
    
    %[pos_target, ~] = calc_targetPosition(othercars,idx_front, idx_rear);   
    pos        = mycar.pos(1);
    pos_frontcar = othercars.car{idx_front}.pos(1);
    pos_rearcar  = othercars.car{idx_rear}.pos(1);
    err_front = pos_frontcar - pos;
    err_rear  = pos - pos_rearcar;
    
    if (err_front > err_rear)&&(err_rear<(mycar.W)*ratio_carLength)
       mycar.vel(1) = mycar.vel(1) + 1000*sim.T;
       return
    elseif (err_front < err_rear)&&(err_front<(mycar.W)*ratio_carLength)
       mycar.vel(1) = mycar.vel(1) - 2000*sim.T;
       return 
    else
       mycar.vel(1) = mycar.vel(1) + 1000*sim.T;
       return       
    end
%----------------------------------------------------------------

end


function [target_pos, target_vel] = calc_targetPosition(othercars,idx_front, idx_rear)

    pos_car1   = othercars.car{idx_front}.pos(1);
    pos_car2   = othercars.car{idx_rear}.pos(1);
    %----
    vel_car1   = othercars.car{idx_front}.vel(1);
    vel_car2   = othercars.car{idx_rear}.vel(1);
    %----
    
    target_pos = (pos_car1 + pos_car2)*0.5;
    target_vel = (vel_car1 + vel_car2)*0.5;
end
