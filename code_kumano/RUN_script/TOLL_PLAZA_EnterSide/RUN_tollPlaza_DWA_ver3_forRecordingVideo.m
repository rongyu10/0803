%ccc
%% MANUALLY COLLECT DRIVING DEMONSTRATIONS
%ccc;
rng(13) % 6
addpath(genpath('../../core'))
addpath(genpath('../../tollplaza'))
addpath(genpath('../../bezier'))
addpath(genpath('../../intelligentDriverModel'))
addpath(genpath('../../dynamicWindow'))
addpath(genpath('../../velocityObstacles_othercars'))
addpath(genpath('../../diverseDriverSetting'))
ccc;
%--- set simulation
road      = init_road_tollplaza();
sim       = init_sim(0.05); % dt = 0.02 [sec]
%------------------
%--- set othercars
othercars  = init_othercars();
desired_velocity = 20000;
ratio_polite_driver = 0.0;
othercars  = init_othercars_IDM(othercars,desired_velocity,ratio_polite_driver);
nr_cars = 20;
X_INTERVAL = 25*10^3;
%othercars    = addcars3_tollplaza(othercars, road.track{1}, nr_cars, X_INTERVAL);
othercars    = addcars3_2lanes_tollplaza(othercars, road.track{1}, nr_cars, X_INTERVAL);
%---------------
%--- set mycar--
ini_vel    = [17000 0];  % [20000 0]
mycar      = init_mycar(get_posintrack(road.track{1}, 2, 0, 2, 0),ini_vel);
%mycar.vel  = [0 0];  % [20000 0]
%mycar.pos  = [175*10^3 15*10^3 0];
myinfo     = get_trackinfo_tollplaza(road, mycar.pos);
% RANGEFINDER SENSOR
mycar.rfs_dist  = 10000;  % 20000
mycar.rfs_deg   = 90;     % 360
mycar.nr_rfs    = 4;      % 25
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
%idx_openGate = [3,4,5,8,9,10,13,14,15];
%idx_openGate = [10,11,12,13,14,15];
%idx_openGate = [1,2,3,4,5,6,7];
idx_openGate = [1,2,3,4,5,6];
othercars = set_othercars_traj(othercars, traj_othercars, idx_openGate);
%---------------------------------

%-----GOAL Setting ---------------
GOAL_GATE = 3;
%---------------------------------

%======= Dynamic Window Approach ============================
FLAG_DynamicWindow     = true;
FLAG_methodDWA         = 1;   % 1: normal DWA, 2: time-varying DWA, 3: DWA with velocity obstacles(under constraction), 5: DWA with SteeringConstrains
PLOT_DWA               = true;
PLOT_COLLISION_AREA    = false;
PLOT_VELOCITY_OBSTACLE = false;

goal      = set_goalgate_position(road.track{4}, GOAL_GATE);
obstacleR = 0;

%[最高速度[mm/s],最高回頭速度[rad/s],最高加減速度[mm/ss],最高加減回頭速度[rad/ss], 速度解像度[mm/s],回頭速度解像度[rad/s]]
Kinematic = [28000, (40.0)/180*pi, 5000, (40.0)/180*pi, 200, (1)/180*pi];
%評価関数のパラメータ [heading,dist,velocity,predictDT]
%evalParam = [60.0,120.0,20.0,2.5];  % [0.1,0.45,0.1,4.0]
area      = [road.xmin, road.xmax, road.ymin, road.ymax];
%============================================================
%---road data-----------------
road_data=[];
%--
roadx = 0:500:(road.xmax);
for i=1:length(roadx)
    road_data=[road_data;roadx(i) 0];        
end
%--
roadx = 0:500:100*10^3;
for i=1:length(roadx)
    road_data=[road_data;roadx(i) 10.5*10^3];        
end
%--
roadx = 275*10^3:500:(road.xmax);
for i=1:length(roadx)
    road_data=[road_data;roadx(i) 75*10^3];        
end
%--
road_data=[road_data; road.track{2}.seg{1}.bd];        
%--
idx_route = [9, 18];
route = [];
for j =1:length(idx_route)
    if j==length(idx_route)
        tmp = traj_othercars.route{idx_route(j),1};
        tmp1= tmp(2:end,:);
    else       
        tmp = traj_othercars.route{idx_route(j),1};
        tmp1= tmp(1:end-1,:);
    end
    route = [route;tmp1];
end
n1 = size(route,1);
route(:,1) = route(:,1) + ones(n1,1)*10*10^3;
route(:,2) = route(:,2) - ones(n1,1)*2*10^3;
road_data=[road_data;route];
%--
roadx = 275*10^3:500:(road.xmax);
for i=1:length(roadx)
    road_data=[road_data;roadx(i) 45*10^3];        
end
%-----------------------------

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
            
            % Intelligent Driver Model
            %othercars = intelligentDriverModel4(othercars,mycar,[],road,sim);
            othercars = ACCModel1(othercars,mycar,[],road,sim);
            
            %othercars  = update_othercars(othercars, sim);
            othercars = update_othercars_along_traj(othercars,sim);
            othercars = get_othercars_info_tollplaza(road,othercars);
            min_intervalTime = 1.8; % 1.8
            othercars = respawn_othercars_tollplaza(othercars,road,sim, min_intervalTime); % added by kumano
            
            if FLAG_methodDWA ==5
                risk_othercars = predict_risk_to_otherCars(mycar, othercars);
            end
            %----   DWA --------------------------------------------------
                  obstacle  =[];
                  for i=1:othercars.n
                     bd = get_carVeryLargeBD(othercars.car{i}.pos);
                     obstacle = [obstacle;bd];
                  end
                  all_obstacle = [road_data;obstacle];
                  
%                   % plot obstacles
%                   if sim.tick==2
%                     f = scatter(all_obstacle(:,1),all_obstacle(:,2));
%                   elseif sim.tick>2
%                     f.XData = all_obstacle(:,1);
%                     f.YData = all_obstacle(:,2);
%                   end

                    if mycar.pos(1)< 230*10^3
                       %evalParam = [20.0,230.0,60.0,2.0];  % [40.0,200.0,20.0,2.0]
                       evalParam  = [30.0,300.0,20.0,1.8];  % [40.0,200.0,20.0,2.0]
                    else
                       evalParam = [150.0,50.0,50.0,2.2];  % [150.0,50.0,150.0,2.5]
                       %evalParam = [10.0,200.0,10.0,2.5];  % [0.1,0.45,0.1,4.0]
                       %GOAL_GATE = 5;
                       %goal      = set_goalgate_position(road.track{4}, GOAL_GATE);
                    end
            if FLAG_DynamicWindow
                 if FLAG_methodDWA ==5
                    [mycar,traj_DWA, eval_DWA] = Control_by_DynamicWindowApproach(mycar,othercars, sim, goal,all_obstacle,obstacleR, Kinematic, evalParam, area, PLOT_DWA, FLAG_methodDWA, risk_othercars);                 
                 else
                    [mycar,traj_DWA, eval_DWA] = Control_by_DynamicWindowApproach(mycar,othercars, sim, goal,all_obstacle,obstacleR, Kinematic, evalParam, area, PLOT_DWA, FLAG_methodDWA);
                 end
            else
                [~,traj_DWA, eval_DWA] = Control_by_DynamicWindowApproach(mycar,othercars, sim, goal,all_obstacle,obstacleR, Kinematic, evalParam, area, PLOT_DWA, FLAG_methodDWA);  
            end            
            %--------------------------------------------------------------

            mycar    = update_mycar(mycar, sim, othercars);

            myinfo     = get_trackinfo_tollplaza(road, mycar.pos);
            ms_update  = etime(clock, clk_update)*1000;
            titlecol = 'w';
            
            % SAVE TRAJ
            %traj = add_traj(traj, mycar, myinfo);
                        
            % TERMINATE CONDITIONS        
            if is_goal(mycar,myinfo,road.track{3},GOAL_GATE)  % added by kumano
                fprintf(1, 'SUCCEEDED!! \n');
                mycar = init_mycar(get_posintrack(road.track{1}, 2, 0, 2, 0),ini_vel); % mod by kumano
                %GOAL_GATE = my_randsample(5,1);
                %goal      = set_goalgate_position(road.track{4}, GOAL_GATE);
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
    PLOT_FUTURE_CARPOSES = 0; % 1
    PLOT_CAR_PATHS       = 1; % 1
    PLOT_RFS             = 0; % 1
%{
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
%}   
    %--- SIMPLER INFORMATION for PLOT---
    strtemp = ['[%.1fSEC][VEL: %.1fKM/H %.1fDEG/S] \n'];
    titlestr = sprintf(strtemp, sim.sec, mycar.vel(1)/10000*36, mycar.vel(2));
    %-----------------------------------
 
    titlefontsize = get_fontsize();
    
    axisinfo = plot_track_tollplaza(road, FILL_LANES);
    plot_goalgate_tollplaza(road.track{4},GOAL_GATE);
    plot_axisinfo(axisinfo,10,10.5,3000,3000);
    col2 = [1 0.5 1] ;
    plot_othercars(othercars, SIMPLECARSHAPE, REALCARSHAPE,col2);
    %--- VELOCITY_OBSTACLE ------
    if PLOT_VELOCITY_OBSTACLE
        maxspeed   = 40*10^3;        
        maxradius  = 40*10^3;
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