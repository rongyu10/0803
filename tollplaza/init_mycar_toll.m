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
mycar.time_mergin_crossing = 2.0; % 自車と他車が交錯する時に取るべき通過時間差（マージン）
mycar.maxtime_judge_front_or_back = 1.0; % 自車と他車の通過時間差がこれ以下なら譲るか譲らないかジャッジする
% --------------------------------------------------------

% setting of following to othercar------------------------------
mycar.time_detect_precedingcar = 1.0;
% --------------------------------------------------------------
