function mycar = chase_othercars(mycar, othercars, idx_front, idx_rear, sim)
% Chase target
persistent first_flag integral_err save_err ini_vel save_idx
if isempty(first_flag)
    first_flag = true;
end

%--- setting initial values ----
if first_flag
   first_flag = false;
   integral_err = 0;
   save_err     = 0;
   ini_vel      = mycar.vel(1);
   save_idx     = idx_front;
end
%-------------------------------

if save_idx ~= idx_front
   integral_err = 0;
   save_err     = 0;
   ini_vel      = mycar.vel(1);
   save_idx     = idx_front;
end

%---PID parameters-------
Kp = 0.8;
Ki = 0.01;
Kd = 0.8;
%------------------------

pos = mycar.pos(1);
vel = mycar.vel(1);

pos_car1 = othercars.car{idx_front}.pos(1);
pos_car2 = othercars.car{idx_rear}.pos(1);

pos_target = (pos_car1 + pos_car2)*0.5;

%--- calc error---
err = pos_target - pos;
integral_err = integral_err + err;
diff_err     = (err - save_err)/sim.T;
%-----------------

if err > 20*10^3
    mycar.vel(1) = 20000;
    first_flag = [];
elseif err < -30*10^3
    mycar.vel(1) = 2000;
    first_flag = [];
else

%--- PID (PI) control ---------
%new_vel = ini_vel + Kp*err;
new_vel = ini_vel + Kp*err + Ki*integral_err;
%new_vel = ini_vel + Kp*err + Ki*integral_err + Kd*diff_err;
%------------------------------

%--- Upper&Lower limitations---
%new_vel = min(25000,new_vel);
%new_vel = max(-4000,new_vel);
%------------------------------

mycar.vel(1) = new_vel;
end

%---- save old error ----
save_err = err;
%------------------------


end
