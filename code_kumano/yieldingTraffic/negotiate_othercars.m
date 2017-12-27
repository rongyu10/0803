function mycar = negotiate_othercars(mycar, othercars, sim, flag_congested)
% Chase target
persistent first_flag first_flag2 flag_reset idx_front idx_rear vel_target ini_vel negotiate_time time_Moving vel_moving start_time
if isempty(first_flag)
    first_flag = true;
    first_flag2= true;
    flag_reset = false;
    negotiate_time = [];
    start_time = [];
end

if sim.tick > 2
    mycar.turnSignal =  'right';
end

%-----Setting for Negotiation -----------------
if flag_congested
    time_MovingToFirstTarget = 2.0; %[sec]
    time_MovingToNextTarget  = 3.0; %[sec]
else
    time_MovingToFirstTarget = 3.0; %[sec]
    time_MovingToNextTarget  = 6.0; %[sec]
end

maxtime_Negotiation = 5.0;      %[sec]
if mycar.lanechange == 1
   maxtime_Negotiation = 10.0;
end
%----------------------------------------------

%--- Moving to the first target----------------------------------
if first_flag
    first_flag = false;
    [~,idx_front1,idx_rear1,~] = get_neighboursCars2(mycar, othercars);
    idx_front = idx_front1;
    idx_rear  = idx_rear1;
    %---- no other cars in front/rear of mycar---
    if (idx_front ==0)||(idx_rear ==0)
       first_flag = true;
       return
    end
    %--------------------------------------------
    [pos_target, ~] = calc_targetPosition(othercars,idx_front, idx_rear);   
    pos        = mycar.pos(1);
    ini_vel    = mycar.vel(1);    
    err        = pos_target - pos;
    time_Moving= time_MovingToFirstTarget;
    d_vel      = err/(time_Moving);   % adjusted from "d_vel = err/time_Moving;" 
    if (ini_vel + d_vel) > 0
        vel_moving = ini_vel + d_vel;        
    else
        vel_moving = 0;
    end
    start_time = sim.sec;
end
%----------------------------------------------------------------

%---- Moving to the next target ---------------------------------
if  flag_reset
    flag_reset = false;
    [~,~,idx_rear1,idx_rear2] = get_neighboursCars2(mycar, othercars);
    idx_front = idx_rear1;
    idx_rear  = idx_rear2;
    %---- no other cars in front/rear of mycar---
    if (idx_front ==0)||(idx_rear ==0)
       flag_reset = true;
       return
    end
    %--------------------------------------------
    [pos_target, ~] = calc_targetPosition(othercars,idx_front, idx_rear);
    pos        = mycar.pos(1);
    ini_vel        = mycar.vel(1);
    err        = pos_target - pos;
    time_Moving= time_MovingToNextTarget;
    d_vel      = err/(time_Moving-1.0);    % adjusted from "d_vel = err/time_Moving;"
    if (ini_vel + d_vel) > 0
        vel_moving = ini_vel + d_vel;
    else
        vel_moving = 0;
    end
    negotiate_time = [];
    start_time = sim.sec;
end
%---------------------------------------------------------------
elapse_time  = sim.sec - start_time;
timer_Moving = time_Moving - elapse_time;

%----------------------------
%if elapse_time < 0.5
if elapse_time < 1.0
   %mycar.vel(1) = (elapse_time/0.5)*vel_moving + (1 - elapse_time/0.5)*ini_vel;
   mycar.vel(1) = (elapse_time)*vel_moving + (1 - elapse_time)*ini_vel;

end
%----------------------------

%---- Monitoring the distance to the target ------------------
[pos_target, vel_target] = calc_targetPosition(othercars,idx_front, idx_rear);    
pos        = mycar.pos(1);
err        = pos_target - pos;

%-- checking position for negotiation -------------------------
%if abs(err) < 2000
if timer_Moving <1.0
    if flag_congested
        vel_final    = vel_target;
    else
        vel_final    = vel_target+1000;        
    end
    
    if (timer_Moving > 0)&&(timer_Moving < 1.0)
       mycar.vel(1) = (1-timer_Moving)*vel_final + timer_Moving*vel_moving;
       %mycar.vel(1) = (1-timer_Moving/2)*vel_final + (timer_Moving/2)*vel_moving;
    else
       mycar.vel(1) = vel_target -500;
    end
    
    if first_flag2
       first_flag2 = false;
       negotiate_time = sim.sec;
    end
end
%-------------------------------------------------------------

%--- Forcing Merging (Urgent Brake)-------
if mycar.pos(1)>180*10^3
   mycar.vel(1) = 0;
   mycar.vel(2) = -3;
   if mycar.pos(3)<-10
       mycar.vel(2) = 0;
   end
end
%-----------------------------------------

%---- Give up negotiation and move to another target------------
if  (~isempty(negotiate_time))&&((sim.sec - negotiate_time)>maxtime_Negotiation)
    flag_reset  = true;
    first_flag2 = true;
end
%---------------------------------------------------------------

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
