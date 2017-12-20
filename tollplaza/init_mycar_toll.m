function mycar = init_mycar_toll(mycar)
% INITIALIZE OTHER CARS FOR TOLLPLAZA (made by yanagihara)

% SETTING OF TOLL ENTERING
mycar.flgPlaza = 0; % 0:before entering plaza, 1:after entering plaza
mycar.startlane = 3;
mycar.goallane = 8;
mycar.pos(2) = 8750 - 3500*(mycar.startlane-1);
mycar.save.lane_idx = mycar.startlane;
mycar.detect_length = 50 * 10^3;
mycar.detect_sidewidth = 3.4 * 10^3;
mycar.max_acceleration = 2.94 * 10^3;
%---------------

% setting of crossing mycar with othercar-----------------------
mycar.x_start_detecting = 50*10^3;
mycar.max_time_dif_intersection = 2;
% min_time_other_intersection = 1.0; % mycar must yield if othercar arrive intersect point within this value
mycar.time_mergin_crossing = 2; % time interval of crossing
% --------------------------------------------------------
