 ccc
%% DRIVING SIMULATOR IN TOLL PLAZA
% 説明-------------------------------------------
% ●入場レーンと目標レーンによって決まる固定パスで、１次元DWAによる加減速度決定
% ●DWAの各候補速度（サンプリング速度）から以下を除外し、残りで評価関数に従って最も評価値の高い速度を選択
%   ・最高速度を上回る
%   ・最低速度を下回る
%   ・他車を急減速させる
%   ・他車と衝突する
% ●評価関数（以下の３つの和）
%　　第1項：自車の現在速度を維持
%　　第2項：自車走行軌道と他車走行軌道の交点通過時間のマージンを取る（全他車で最も近いものを評価）
%　　第3項：今回は未使用
% -----------------------------------------------
ccc;
addpath(genpath('./core'))
addpath(genpath('./tollplaza'))
addpath(genpath('./bezier'))
addpath(genpath('./intelligentDriverModel'))
addpath(genpath('./turnSignal'))

%--- SET SIMULATION -----------------------------------------------------------------------------------------------
road      = init_road_tollplaza_IN();
dt = 0.1;
sim       = init_sim(dt); % dt = 0.02 [sec]
%------------------------------------------------------------------------------------------------------------------

%-- MAKE LANECHANGE PATH-------------------------------------------------------------------------------------------
dx = 175*10^3/3; % x-cood.interval of control points for bezier curve
for i = 1:15 % i:start lane j:goal lane
    for j = 1:3
        ctlPt = [100*10^3 12250 - 3500 * j; 100*10^3 + dx 12250 - 3500 * j; 100*10^3 + 2*dx 77.5*10^3 - 5*10^3*i; 100*10^3 + 3*dx 77.5*10^3 - 5*10^3*i];
        [laneChangePath{i,j}, lengthP{i,j}] = bezierCurve(ctlPt);
    end
end
%------------------------------------------------------------------------------------------------------------------

%--- SET OTHERCARS -------------------------------------------------------------------------------------------------
othercars  = init_othercars();
nr_cars    = 46; % total number of cars
othercars.npl = 20; % number of cars per lane
othercars  = addcars_tollplaza_IN_cross1lane(othercars, road.track{1}, nr_cars, laneChangePath);
%othercars  = addcars_tollplaza_IN(othercars, road.track{1}, nr_cars, laneChangePath);
%othercars  = addcars_tollplaza_IN_lower5(othercars, road.track{1}, nr_cars, laneChangePath);

othercars.step_TTC = sim.T;
othercars.detect_rect_length = 30 * 10^3;
othercars.detect_rect_sidewidth = 0.1 * 10^3;
othercars.max_acceleration = 2.94 * 10^3;
othercars.detect_length = 50 * 10^3;
%--------------------------------------------------------------------------------------------------------------------

%---- SET MYCAR -----------------------------------------------------------------------------------------------------
ini_vel    = [17500 0]; % 20000 mm/s = 72 km/h
ini_pos    = [-25000 5250 0];
mycar      = init_mycar(ini_pos, ini_vel);
myinfo     = get_trackinfo_tollplaza(road, mycar.pos, othercars);
mycar.flgPlaza = 0; % 0:before entering plaza, 1:after entering plaza
mycar.startlane = 3;
mycar.goallane = 8;
mycar.pos(2) = 8750 - 3500*(mycar.startlane-1);
mycar.save.lane_idx = mycar.startlane;
mycar.detect_length = 100 * 10^3;
mycar.detect_sidewidth = 3.4 * 10^3;
mycar.max_acceleration = 2.94 * 10^3;

% setting of crossing to othercar-------------------------
mycar.x_start_detecting = 50*10^3; % 料金所プラザ内で交錯する他者を観測し始める地点（ｘ座標）
mycar.time_margin_crossing = 2.0; % 自車と他車が交錯する時に取るべき通過時間差（マージン）
mycar.invadepoint = [];
mycar.othercars_travel_area_side = 3.4 * 10^3;
% --------------------------------------------------------

% setting of DWA -----------------------------------------
RangeDWA=[0 20000 mycar.max_acceleration*sim.T 40]; %[最低速度(mm/s)　最高速度(mm/s)　速度レンジ(m/s)　速度解像度(分割数)]
ParamDWA=[1.0 1.5 1.0]; %[現在の速度を維持する項　前方車との予測通過時間差の項　後方車を急減速させない項]　
mycar.let_othercar_decele = 2.94 * 10^3; % 他車に与える減速度の許容値
% --------------------------------------------------------

% setting of following to othercar------------------------
mycar.time_detect_precedingcar = 0.1;
% --------------------------------------------------------
% --------------------------------------------------------------------------------------------------------------------


% PARAMETER OF INTELLIGENT DRIVING MODEL(from Kesting "Enhanced intelligent ..." (2010))--------------------
idm.v0 = 17500; % desired velocity
idm.T = 1.5; % Safe time headway
idm.a = 1400; % maximum acceleration
idm.b = 2000; %desired deceleration
idm.delta = 4; %acceleration exponent
idm.s0 = 2000; % minimum distance
idm.l = 4000; % vehicle length
idm.coolness = 0.99;
%-----------------------------------------------------------


% INITIALIZE FIGURE-----------
figsz     = [1 4 8 4]/10;
figtitle  = 'GUI-GUI CONTROL SIMULATOR at TOLL PLAZA';
axespos   = [0.03, 0.02, 0.95, 0.84]; 
fig       = get_fig(figsz, figtitle, axespos);
set(gcf,'Color', [0.1, 0.25, 0.2] ); hold on;
ms_update = 0; ms_plot = 0;
%-----------------------------


%-- FLAG INTERACTION ---------
FLAG_LANECHANGE  = false;
FLAG_UPDATE_RFS = false;
%-----------------------------


%--parameter for getting trajectory data
plot_mycardec = [];
plot_othercardec = [];
time_plot_mycardec = 0;
%-----------------------------------


%--PLOTING MODE-------------
PLOT_MYCAR_DETECTING_AREA = 1;
%---------------------------



% RUN
% INITIALIZE SAVER
traj = init_traj(road.track{1}, mycar, othercars);
while sim.flag && ishandle(fig)
    % KEYBOARD CONTROLLER
    switch key_pressed 
        case ''
        case {'leftarrow', 'semicolon'}
            othercars.car{2}.vel(1) = othercars.car{2}.vel(1)-5000;
            %mycar.vel(1) = mycar.vel(1) - 5000;
            fprintf(2, 'MANUAL: car[2] decelerate\n');
        case {'rightarrow', 'quote'}
            othercars.car{2}.vel(1) = othercars.car{2}.vel(1)+5000;
            %mycar.vel(1) = mycar.vel(1) + 5000;
            fprintf(2, 'MANUAL: car[2] accelerate\n');
        case {'uparrow', 'leftbracket'}
            % change the goal(target) lane
            %mycar.vel(1) = mycar.vel(1)+5000;
            if mycar.pos(1) < mycar.x_start_detecting && mycar.goallane ~= 1
                mycar.goallane = mycar.goallane - 1;
            end
        case {'downarrow', 'slash'}
            % change the goal(target) lane
            %mycar.vel(1) = mycar.vel(1)-5000;
            if mycar.pos(1) < mycar.x_start_detecting && mycar.goallane ~= 15
                mycar.goallane = mycar.goallane + 1;
            end
        case 'space'
            mycar.vel = [0 0];  
        case {'1', '2', '3', '4', '5', '6'}
            nr_lane = str2num(key_pressed);
            mycar = set_mycar(mycar, get_posintrack(road.track{1}, 1, 0, nr_lane, 0), [ini_vel 0]);
        case 's' % MAKE A CAR GO STRAIGHT
            mycar.pos(3) = 0; mycar.vel(2) = 0;
        case 'p'
            if isequal(sim.mode, 'RUN'), sim.mode = 'PAUSE'; 
            elseif isequal(sim.mode, 'PAUSE'), sim.mode = 'RUN'; end
        case 'q' 
            sim.mode = 'QUIT';
        case 'l'
            % LANE CHANGING (add by kumano)
            FLAG_LANECHANGE = true;
        case 'a'
            % Checking for Intelligetn Driver Model (add by kumano)
            othercars.car{1}.vel(1) = othercars.car{1}.vel(1) - 2000;
        case 'z'
            % Checking for Intelligetn Driver Model (add by kumano)
            [~,idx_rear] = get_neighboursCars(mycar, othercars);
            if idx_rear~=0
                othercars.car{idx_rear}.vel(1) = othercars.car{idx_rear}.vel(1) - 2000;
            end
        case 'x'
            mycar.vel(2) = mycar.vel(2) - 5;
        case 'c'
            mycar.vel(2) = mycar.vel(2) + 5;
        otherwise 
            fprintf(2, 'KEY[%s] UNDEFINED. \n', key_pressed);
    end
    key_pressed = ''; 
    
    % SIMULATE 
    switch sim.mode 
        case 'RUN'
            % UPDATE
            clk_update = clock;
            sim        = update_sim(sim);
            othercars = respawn_othercars_tollplaza(othercars,road);
            
            % update speed and position of othercars
            othercars = calculate_velocity_othercars_tollPlaza_IN(othercars, sim, mycar, idm, laneChangePath);
            
            % update speed and position of mycar
            mycar = calculate_velocity_mycar_tollPlaza_IN_DWA6(mycar, sim, othercars, idm, laneChangePath, RangeDWA, ParamDWA);
            
            mycar = update_mycar(mycar, sim, othercars, FLAG_UPDATE_RFS);
            othercars = update_othercars(othercars, sim);
            
            myinfo     = get_trackinfo_tollplaza(road, mycar.pos, othercars);
            ms_update  = etime(clock, clk_update)*1000;
            titlecol = 'w';
            
            % SAVE TRAJppp
            %traj = add_traj(traj, mycar, myinfo);
                        
            % TERMINATE CONDITIONS        
%             if is_goal(mycar,myinfo,road.track{3},2)  % added by kumano
%                 fprintf(1, 'SUCCEEDED!! \n');
%                 mycar = init_mycar(get_posintrack(road.track{1}, 1, 0, 2, 0),ini_vel); % mod by kumano
%                 FLAG_LANECHANGE = false;
%              end
            
            if mycar.pos(1) > 320*10^3
                fprintf(1, 'SUCCEEDED!! \n');              
                sim.mode = 'QUIT';
            elseif is_insidetrack(myinfo) == 0 && (mycar.pos(2) < 0 || mycar.pos(2) > 10500) 
                fprintf(2, 'OUTSIDE THE TRACK. \n');
                sim.mode = 'QUIT';
            end
            if is_carcrashed3(mycar, othercars) % mod by kumano
                fprintf(2, 'COLLISION OCCURRED. \n');
                sim.mode = 'QUIT';
            end
        case 'PAUSE'
            titlecol = 'c';
        case 'QUIT'
            sim.flag = 0;
            titlecol = 'r';
    end
   
    % PLOT
    clk_plot = clock;
    FILL_LANES           = 1; % 1
    SIMPLECARSHAPE       = 1; % 0(描画処理が重い場合は SIMPLECARSHAPE=1, REALCARSHAPE=0とする)
    REALCARSHAPE         = 0; % 1 
    PLOT_FUTURE_CARPOSES = 0; % 1
    PLOT_CAR_PATHS       = 0; % 1
    PLOT_RFS             = 0; % 1
%     strtemp = ['[%.1fSEC][UPDATE:%.1fMS+PLOT:%.1fMS] ' ...
%         '[VEL: %.1fKM/H %.1fDEG/S] \n' ...
%         '[%dSEG-%dLANE] / [LANE-DEV DIST:%.1fMM DEG:%.1fDEG] \n' ...
%         '[LEFT:%.2fM-CENTER:%.2fM-RIGHT:%.2fM]\n' ...
%         '[#SAVE: %d]'];
%     titlestr = sprintf(strtemp, sim.sec, ms_update, ms_plot ...
%         , mycar.vel(1)/10000*36, mycar.vel(2) ...
%         , myinfo.seg_idx, myinfo.lane_idx, myinfo.lane_dev, myinfo.deg ...
%         , myinfo.left_fb_dists(1)/1000, myinfo.center_fb_dists(1)/1000 ...
%         , myinfo.right_fb_dists(1)/1000 ...
%         , traj.data.n);
    strtemp = ['[%.1fSEC][UPDATE:%.1fMS+PLOT:%.1fMS = %.1fMS] \n' ...
        '[Mycar VEL:%5.0fMM/S = %.1fKM/H] \n' ...
        '[Car(21) VEL:%5.0fMM/S = %.1fKM/H, DISTtoCar(20) is %5.0fMM, THWtoCar(20) is %1.2fSEC]'];
    titlestr = sprintf(strtemp, sim.sec, ms_update, ms_plot, ms_update + ms_plot ...
        , mycar.vel(1), mycar.vel(1)/10000*36 ...
        , othercars.car{21}.vel(1), othercars.car{21}.vel(1)/10000*36, norm(othercars.car{20}.pos(1:2) - othercars.car{21}.pos(1:2)), norm(othercars.car{2}.pos(1:2) - othercars.car{3}.pos(1:2))/othercars.car{3}.vel(1));
    titlefontsize = get_fontsize();
    
    axisinfo = plot_track_tollplaza(road, FILL_LANES);
    plot_axisinfo_tollplaza(axisinfo);
    plot_arrow_selectlane_IN(mycar.goallane);
    
    plot_mycar(mycar, PLOT_FUTURE_CARPOSES, PLOT_CAR_PATHS, SIMPLECARSHAPE, REALCARSHAPE, PLOT_RFS);
    plot_mycar_path_in_plaza(laneChangePath, mycar);
    plot_mycar_detecting_area(mycar);
    if ~isempty(mycar.invadepoint)
        plot_invadepoint(mycar.invadepoint);
    end
    plot_othercars_text(othercars, SIMPLECARSHAPE, REALCARSHAPE);
    %----
    plot_title(titlestr, titlecol, titlefontsize);
    drawnow;
    
    ms_plot = etime(clock, clk_plot)*1000;
    
end
fprintf(2, 'SIMULATION TERMINATED \n');

%%