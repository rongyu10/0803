function mycar = hazard_lamp(mycar,myinfo,sim,id_lane)
% SET HAZARD LAMP after lane changing
persistent first_flag signal_flag signal_time

if isempty(first_flag)
    first_flag = true;
    signal_flag = 0;
end

%-----------------------
if isfield(mycar,'turnSignal')==0
   return
end
if myinfo.lane_idx ~= id_lane
    first_flag = true;
    signal_flag = 0;
end
%-----------------------

%--- turn ON hazard lamp-----
if first_flag
   if myinfo.lane_idx == id_lane
      first_flag = false;
      mycar.turnSignal = 'hazard';
      signal_flag = 1;
      signal_time = sim.sec;
   end
end
%-----------------------------

%--- turn OFF hazard lamp-----
if (signal_flag == 1)&&((sim.sec - signal_time)> 3)
    signal_flag = 0;
    mycar.turnSignal = '';
end
%-----------------------------
end
