function [othercars, dec] = update_othercars_mycar_intelligent_merge(othercars, sim, track, mycar, car_nr, idm) % added by yanagihara

% PARAMETER OF INTELLIGENT DRIVING MODEL---------------------
v0 = idm.v0; % desired velocity
T = idm.T; % Safe time headway
a = idm.a; % maximum acceleration
b = idm.b; %desired deceleration
delta = idm.delta; %acceleration exponent
s0 = idm.s0; % minimum distance
l = idm.l; % vehicle length
%============================================================


for i = 1:othercars.n
    
    front_num = i + 1;
    if mod(front_num, othercars.npl) == 1
        front_num = front_num - othercars.npl;
    end
    
    if mod(i, othercars.npl) ~= 0
        A1 = othercars.car{i}.vel(1)/v0;
        
        if i == car_nr % if the calculation target car is just behind of mycar
            if mycar.pos(1) - othercars.car{i}.pos(1) < 0 % if there is no other car front of this car
                A3 = track.xmax - othercars.car{i}.pos(1) + mycar.pos(1) - l;
            else
                A3 = mycar.pos(1) - othercars.car{i}.pos(1) - l;
            end
            A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - mycar.vel(1))/2/sqrt(a*b))/A3;
        else
            if othercars.car{front_num}.pos(1) - othercars.car{i}.pos(1) < 0 % if there is no other car front of this car
                A3 = track.xmax - othercars.car{i}.pos(1) + othercars.car{front_num}.pos(1) - l;
            else
                A3 = othercars.car{front_num}.pos(1) - othercars.car{i}.pos(1) - l;
            end
            A2 = (s0 + othercars.car{i}.vel(1)*T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - othercars.car{front_num}.vel(1))/2/sqrt(a*b))/A3;
        end
        
        if a*(1 - A1^delta - A2^2) < -3000
            othercars.car{i}.vel(1) = othercars.car{i}.vel(1) - 3000*sim.T;
            dec = 3000;
        else
            othercars.car{i}.vel(1) = othercars.car{i}.vel(1) + a*(1 - A1^delta - A2^2)*sim.T;
            dec = -a*(1 - A1^delta - A2^2);
        end
        if othercars.car{i}.vel(1) < 0
            othercars.car{i}.vel(1) = 0;
        end
    end
    
    
    othercars.car{i}.pos ...
        = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T);
    
    
    othercars.car{i}.bd ...
        = get_carshape(othercars.car{i}.pos ...
        , othercars.car{i}.W, othercars.car{i}.H);
end
