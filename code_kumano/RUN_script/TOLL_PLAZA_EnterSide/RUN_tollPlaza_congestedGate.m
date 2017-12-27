%ccc
%% MANUALLY COLLECT DRIVING DEMONSTRATIONS
%ccc;
rng(8) % 6
addpath(genpath('../../core'))
addpath(genpath('../../tollplaza'))
addpath(genpath('../../bezier'))
addpath(genpath('../../intelligentDriverModel'))
addpath(genpath('../../diverseDriverSetting'))
addpath(genpath('../../velocityObstacles_othercars'))
ccc;
%--- set simulation
road      = init_road_tollplaza();
sim       = init_sim(0.05); % dt = 0.02 [sec]
%-----------------
%--- set othercars
othercars  = init_othercars();
desired_velocity = 20000;
ratio_polite_driver = 0.0;
othercars  = init_othercars_IDM(othercars,desired_velocity,ratio_polite_driver);
nr_cars = 50;
othercars    = addcars3_tollplaza(othercars, road.track{1}, nr_cars);
%---------------
%--- set mycar--
ini_vel    = [20000 0];  % [20000 0]
mycar      = init_mycar(get_posintrack(road.track{1}, 2, 0, 2, 0),ini_vel);
%mycar.vel  = [0 0];  % [20000 0]
%mycar.pos  = [175*10^3 15*10^3 0];
myinfo     = get_trackinfo_tollplaza(road, mycar.pos);
% RANGEFINDER SENSOR
mycar.rfs_dist  = 10000;  % 20000
mycar.rfs_deg   = 90;     % 360
mycar.nr_rfs    = 3;      % 25
mycar.rfs_degs  = linspace(-mycar.rfs_deg/2, mycar.rfs_deg/2, mycar.nr_rfs);
mycar.rfs_dists = mycar.rfs_dist*ones(1, mycar.nr_rfs);
%---------------

% INITIALIZE FIGURE
figsz     = [1 4 8 4]/10;
figtitle  = 'GUI-GUI CONTROL SIMULATOR at TOLL PLAZA';
axespos   = [0.03, 0.02, 0.95, 0.84]; 
fig       = get_fig(figsz, figtitle, axespos);
set(gcf,'Color', [0.1, 0.25, 0.2] ); hold on;
ms_update = 0; ms_plot = 0;

%-- FLAG INTERACTION ---------
FLAG_INTERACTION = true;
%-----------------------------

%---- OTHTER CARS TRAJECTORIES----
%traj_othercars = get_othercars_trajectories_80m_tollplaza();
traj_othercars = get_othercars_trajectories_60m_tollplaza();
idx_openGate = [3,4,5,8,9,10,13,14,15];
othercars = set_othercars_traj_congestedGate(othercars, traj_othercars, idx_openGate);
%---------------------------------

%---- LOAD othercars DATA --------
load('../../trafficData/congestedGate.mat')
%---------------------------------

%--- VELOCITY_OBSTACLE ---------
PLOT_VELOCITY_OBSTACLE = false;
%-------------------------------

%-----GOAL Setting ---------------
GOAL_GATE = 3;
%---------------------------------

% RUN
% INITIALIZE SAVER
traj = init_traj(road.track{1}, mycar, othercars);
while sim.flag && ishandle(fig)
    % KEYBOARD CONTROLLER
    switch key_pressed 
        case ''
        case {'leftarrow', 'semicolon'}
            %mycar.vel(2) = mycar.vel(2)+20;
             mycar.vel(2) = mycar.vel(2)+5;
        case {'rightarrow', 'quote'}
            %mycar.vel(2) = mycar.vel(2)-20;
            mycar.vel(2) = mycar.vel(2)-5;
        case {'uparrow', 'leftbracket'}
            %mycar.vel(1) = mycar.vel(1)+5000; % 10000 mm/s = 36 km/h
            mycar.vel(1) = mycar.vel(1)+1000;
        case {'downarrow', 'slash'}
            %mycar.vel(1) = mycar.vel(1)-5000; % 10000 mm/s = 36 km/h
            mycar.vel(1) = mycar.vel(1)-1000;
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
        case 'a'
            % Checking for Intelligent Driver Model (add by kumano)
            othercars.car{1}.vel(1) = othercars.car{1}.vel(1) - 2000;
        case 'z'
            % Checking for Intelligent Driver Model (add by kumano)
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
            
            % Intelligent Driver Model
            %othercars = intelligentDriverModel4(othercars,mycar,[],road,sim);
            v0_gate = 2300;
            othercars = ACCModel1(othercars,mycar,[],road,sim,'enter',v0_gate);
            %othercars = ACCModel1_old(othercars,mycar,[],road,sim);
            
            %othercars  = update_othercars(othercars, sim);
            othercars = update_othercars_along_traj(othercars,sim);
            othercars = get_othercars_info_tollplaza(road,othercars);
            othercars = respawn_othercars_tollplaza(othercars,road,sim); % added by kumano
            
            mycar    = update_mycar(mycar, sim, othercars);

            myinfo     = get_trackinfo_tollplaza(road, mycar.pos);
            %[idx_frontL,idx_frontR,idx_rearL,idx_rearR] = get_neighboursCars_tollplaza(mycar, othercars);
            risk_othercars = predict_risk_to_otherCars(mycar, othercars);

            ms_update  = etime(clock, clk_update)*1000;
            titlecol = 'w';
            
            % SAVE TRAJ
            %traj = add_traj(traj, mycar, myinfo);
                        
            % TERMINATE CONDITIONS        
            if is_goal(mycar,myinfo,road.track{3},GOAL_GATE)  % added by kumano
                fprintf(1, 'SUCCEEDED!! \n');
                mycar = init_mycar(get_posintrack(road.track{1}, 2, 0, 2, 0),ini_vel); % mod by kumano
                GOAL_GATE = my_randsample(15,1);
            end
            if is_insidetrack(myinfo) == 0
                fprintf(2, 'OUTSIDE THE TRACK. \n');
                mycar = init_mycar(get_posintrack(road.track{1}, 2, 0, 2, 0),ini_vel); % mod by kumano
            end
            %if is_carcrashed(myinfo)
            if is_carcrashed3(mycar,othercars) % mod by kumano
                fprintf(2, 'COLLISION OCCURRED. \n');
                othercars = delete_crashedcar(mycar,othercars);
                mycar = init_mycar(get_posintrack(road.track{1}, 2, 0, 2, 0),ini_vel); % mod by kumano
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
    PLOT_CAR_PATHS       = 1; % 1
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
    titlefontsize = get_fontsize();
    
    axisinfo = plot_track_tollplaza(road, FILL_LANES);
    plot_goalgate_tollplaza(road.track{4},GOAL_GATE);
    plot_axisinfo(axisinfo,10,10.5,3000,3000);
    plot_othercars(othercars, SIMPLECARSHAPE, REALCARSHAPE);
    %test_plot_othercars(othercars, SIMPLECARSHAPE, REALCARSHAPE,idx_frontL,idx_frontR,idx_rearL,idx_rearR);
    %--- VELOCITY_OBSTACLE ------
    if PLOT_VELOCITY_OBSTACLE
        maxspeed   = 30*10^3;        
        maxradius  = 30*10^3;
        plot_velocityObstacle_othercars(mycar, othercars, maxradius, maxspeed);
    end
    %----------------------------    
    plot_mycar(mycar, PLOT_FUTURE_CARPOSES, PLOT_CAR_PATHS, SIMPLECARSHAPE, REALCARSHAPE, PLOT_RFS);
    %plot_traj(traj);
    %plot_othercars_traj_tollplaza(traj_othercars);
    %----
    plot_title(titlestr, titlecol, titlefontsize);
    drawnow;
    ms_plot = etime(clock, clk_plot)*1000;
    
end
fprintf(2, 'SIMULATION TERMINATED \n');

%%