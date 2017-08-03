function mycar = negotiate_othercars(mycar, othercars, idx_front, idx_rear)
% Chase target
persistent first_flag integral_err save_err ini_vel
if isempty(first_flag)
    first_flag = true;
end

%--- setting initial values ----
if first_flag
   first_flag = false;
   integral_err = 0;
   save_err     = 0;
   ini_vel      = mycar.vel(1);
end
%-------------------------------

%---PID parameters-------
Kp = 2.0;
Ki = 0.15;
Kd = 0.1;
%------------------------

pos = mycar.pos(1);
vel = mycar.vel(1);

pos_car1 = othercars.car{idx_front}.pos(1);
pos_car2 = othercars.car{idx_rear}.pos(1);

pos_target = (pos_car1 + pos_car2)*0.5;

%--- calc error---
err = pos_target - pos;
integral_err = integral_err + err;
diff_err     = err - save_err;
%-----------------

%--- PID (PI) control ---------
%new_vel = ini_vel + Kp*err;
new_vel = ini_vel + Kp*err + Ki*integral_err;
%new_vel = ini_vel + Kp*err + Ki*integral_err + Kd*diff_err;
%------------------------------

%--- Upper&Lower limitations---
new_vel = min(18000,new_vel);
%new_vel = max(-4000,new_vel);
%------------------------------

mycar.vel(1) = new_vel;

%---- save old error ----
save_err = err;
%------------------------
end
