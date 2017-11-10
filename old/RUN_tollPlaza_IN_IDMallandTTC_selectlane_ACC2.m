 ccc
%% MANUALLY COLLECT DRIVING DEMONSTRATIONS
ccc;
addpath(genpath('./core'))
addpath(genpath('./tollplaza'))
addpath(genpath('./bezier'))
addpath(genpath('./intelligentDriverModel'))
addpath(genpath('./turnSignal'))
%--- set simulation
road      = init_road_tollplaza_IN();
sim       = init_sim(0.1); % dt = 0.02 [sec]
%------------------
%--- set othercars
othercars  = init_othercars();
nr_cars    = 46; % total number of cars (1st:20cars, 2nd:6cars, 3rd:20cars)
othercars.npl = 20; % number of cars per lane (1st and 3rd lane)
othercars  = addcars_tollplaza_IN(othercars, road.track{1}, nr_cars);
load('othercars_over3G');

%---------------
%--- set mycar--
ini_vel    = [15000 0]; % 20000 mm/s = 72 km/h
ini_pos    = [-135000 5250 0];
mycar      = init_mycar(ini_pos, ini_vel);
myinfo     = get_trackinfo_tollplaza(road, mycar.pos, othercars);
% SETTING OF TOLL ENTERING
mycar.flgPlaza = 0; % 0:before entering plaza, 1:after entering plaza
mycar.startlane = 3;
mycar.selectlane = 8;
mycar.pos(2) = 8750 - 3500*(mycar.startlane-1);
mycar.front_nr = 0; % carID in front of mycar
mycar.rear_nr = 0; % carID behind mycar
mycar.save.lane_idx = mycar.startlane;
mycar.flgIDM = 0;
mycar.squareX = zeros(1,21);
mycar.squareY = zeros(1,21);
%---------------

% PARAMETER OF INTELLIGENT DRIVING MODEL--------------------
idm.v0 = 15000; % desired velocity
idm.T = 1.5; % Safe time headway
idm.a = 1000; % maximum acceleration
idm.b = 3000; %desired deceleration
idm.delta = 4; %acceleration exponent
idm.s0 = 2000; % minimum distance
idm.l = 4000; % vehicle length
%============================================================

% INITIALIZE FIGURE
figsz     = [1 4 8 4]/10;
figtitle  = 'GUI-GUI CONTROL SIMULATOR at TOLL PLAZA';
axespos   = [0.03, 0.02, 0.95, 0.84]; 
fig       = get_fig(figsz, figtitle, axespos);
set(gcf,'Color', [0.1, 0.25, 0.2] ); hold on;
ms_update = 0; ms_plot = 0;


%-- FLAG INTERACTION ---------
FLAG_INTERACTION = true;
FLAG_LANECHANGE  = false;
%-----------------------------

% MAKE LANECHANGE PATH
dx = 175*10^3/3; % x-cood.interval of control points for bezier curve
for i = 1:15 % i:start lane j:goal lane
    for j = 1:3
        ctlPt = [100*10^3 12250 - 3500 * j; 100*10^3 + dx 12250 - 3500 * j; 100*10^3 + 2*dx 77.5*10^3 - 5*10^3*i; 100*10^3 + 3*dx 77.5*10^3 - 5*10^3*i];
        [laneChangePath{i,j}, lengthP{i,j}] = bezierCurve(ctlPt);
    end
end



% RUN
% INITIALIZE SAVER
traj = init_traj(road.track{1}, mycar, othercars);
while sim.flag && ishandle(fig)
    % KEYBOARD CONTROLLER
    switch key_pressed 
        case ''
        case {'leftarrow', 'semicolon'}
            mycar.vel(1) = mycar.vel(1)-5000;
        case {'rightarrow', 'quote'}
            mycar.vel(1) = mycar.vel(1)+5000;
        case {'uparrow', 'leftbracket'}
            % change the goal(target) lane
            %mycar.vel(1) = mycar.vel(1)+5000;
            if mycar.pos(1) < 100*10^3 && mycar.selectlane ~= 1
                mycar.selectlane = mycar.selectlane - 1;
            end
        case {'downarrow', 'slash'}
            % change the goal(target) lane
            %mycar.vel(1) = mycar.vel(1)-5000;
            if mycar.pos(1) < 100*10^3 && mycar.selectlane ~= 15
                mycar.selectlane = mycar.selectlane + 1;
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
            othercars  = respawn_othercars_tollplaza(othercars,road,sim);
            
            % update speed and position of mycar (included merging and IDM)
            [mycar, othercars] = update_control_mycar_IN_IDMallandTTC_norfs_ACC2(mycar, sim, othercars, idm, laneChangePath);
            
            % update speed and position of othercars (included merging and IDM)
            [othercars, mycar]  = update_control_othercars_mycar_IN_TTCandIDMall_manual(othercars, sim, mycar, idm, laneChangePath, lengthP, FLAG_LANECHANGE);
            
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
                key_pressed = 'p';
                mycar = init_mycar(get_posintrack(road.track{1}, 1, 0, 2, 0),ini_vel); % mod by kumano
                mycar.flgPlaza = 0; % 0:before entering plaza, 1:after entering plaza
                mycar.startlane = 3;
                mycar.selectlane = 8;
                mycar.front_nr = 0; % carID in front of mycar
                mycar.rear_nr = 0; % carID behind mycar
                mycar.save.lane_idx = mycar.startlane;
                FLAG_LANECHANGE = false;
                sim.mode = 'QUIT';
                clear update_control_mycar_merge_intelligent
            elseif is_insidetrack(myinfo) == 0 && (mycar.pos(2) < 0 || mycar.pos(2) > 10500) 
                fprintf(2, 'OUTSIDE THE TRACK. \n');
                mycar = init_mycar(get_posintrack(road.track{1}, 1, 0, 2, 0),ini_vel); % mod by kumano
                mycar.flgPlaza = 0; % 0:before entering plaza, 1:after entering plaza
                mycar.startlane = 3;
                mycar.selectlane = 8;
                mycar.front_nr = 0; % carID in front of mycar
                mycar.rear_nr = 0; % carID behind mycar
                mycar.save.lane_idx = mycar.startlane;
                FLAG_LANECHANGE = false;
            end
            %if is_carcrashed(myinfo)
            if is_carcrashed2(mycar) % mod by kumano
                fprintf(2, 'COLLISION OCCURRED. \n');
                mycar = init_mycar(get_posintrack(road.track{1}, 1, 0, 2, 0),ini_vel); % mod by kumano
                mycar.flgPlaza = 0; % 0:before entering plaza, 1:after entering plaza
                mycar.startlane = 3;
                mycar.selectlane = 8;
                mycar.front_nr = 0; % carID in front of mycar
                mycar.rear_nr = 0; % carID behind mycar
                mycar.save.lane_idx = mycar.startlane;
                FLAG_LANECHANGE = false;
                clear update_control_mycar_merge_intelligent
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
    PLOT_FUTURE_CARPOSES = 1; % 1
    PLOT_CAR_PATHS       = 0; % 1
    PLOT_RFS             = 0; % 1
    strtemp = ['[%.1fSEC][UPDATE:%.1fMS+PLOT:%.1fMS] ' ...
        '[VEL: %.1fKM/H %.1fDEG/S] \n' ...
        '[%dSEG-%dLANE] / [LANE-DEV DIST:%.1fMM DEG:%.1fDEG] \n' ...
        '[LEFT:%.2fM-CENTER:%.2fM-RIGHT:%.2fM]\n' ...
        '[#SAVE: %d]'];
    titlestr = sprintf(strtemp, sim.sec, ms_update, ms_plot ...
        , mycar.vel(1)/10000*36, mycar.vel(2) ...
        , myinfo.seg_idx, myinfo.lane_idx, myinfo.lane_dev, myinfo.deg ...
        , myinfo.left_fb_dists(1)/1000, myinfo.center_fb_dists(1)/1000 ...
        , myinfo.right_fb_dists(1)/1000 ...
        , traj.data.n);
%     strtemp = '[%.1fSEC][UPDATE:%.1fMS+PLOT:%.1fMS] ';
%     titlestr = sprintf(strtemp, sim.sec, ms_update, ms_plot);
    titlefontsize = get_fontsize();
    
    axisinfo = plot_track_tollplaza(road, FILL_LANES);
    plot_axisinfo_tollplaza(axisinfo);
    plot_arrow_selectlane_IN(mycar.selectlane);
    plot_othercars(othercars, SIMPLECARSHAPE, REALCARSHAPE);
    plot_mycar(mycar, PLOT_FUTURE_CARPOSES, PLOT_CAR_PATHS, SIMPLECARSHAPE, REALCARSHAPE, PLOT_RFS);
    plot_mycar_path_in_plaza(laneChangePath, mycar);
    plot_mycar_detecting_area(mycar);
    %----
    plot_title(titlestr, titlecol, titlefontsize);
    drawnow;
    
    ms_plot = etime(clock, clk_plot)*1000;
    
end
fprintf(2, 'SIMULATION TERMINATED \n');

%%