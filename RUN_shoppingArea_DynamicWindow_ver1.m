ccc
%% MANUALLY COLLECT DRIVING DEMONSTRATIONS
ccc;
rng(9)
warning('off','all')

addpath(genpath('./core'))
addpath(genpath('./pedestrian'))
addpath(genpath('./dynamicWindow'))
addpath(genpath('./velocityObstacles'))
%========== Simulator Settings ================================ 
% INITIALIZE ENVIRONMENT
TRACK_TYPE = 'SHORT_STRAIGHT';
NR_LANE    = 1;
LANE_WIDTH = 6500; % LANE WIDTH
track      = init_track(NR_LANE, LANE_WIDTH, TRACK_TYPE);
sim        = init_sim(0.2); % dt = 0.02 [sec]
othercars  = init_othercars();
nr_cars    = 0;
ini_vel    = [0 0];
othercars  =  addcars_shoppingArea(othercars, track, nr_cars);
mycar      = init_mycar(get_posintrack(track, 1, 0, 1, 0),ini_vel);
%--pedestrians
nr_people = 6;  % Number of Pedestrians
pedestrians= init_pedestrians();
pedestrians=  addpedestrians(pedestrians, track, nr_people);

% INITIALIZE FIGURE
figsz     = [1 4 8 4]/10;
figtitle  = 'GUI-GUI CONTROL SIMULATOR in Shopping Area';
axespos   = [0.03, 0.02, 0.95, 0.84]; 
fig       = get_fig(figsz, figtitle, axespos);
set(gcf,'Color', [0.1, 0.25, 0.2] ); hold on;
ms_update = 0; ms_plot = 0;
%-----------------------------

%============================================================
%======= Dynamic Window Approach ============================
FLAG_DynamicWindow     = true;
FLAG_methodDWA         = 1;   % 1: normal DWA, 2: time-varying DWA, 3: DWA with velocity obstacles(under constraction) 
PLOT_DWA               = true;
PLOT_COLLISION_AREA    = false;
PLOT_VELOCITY_OBSTACLE = true;
goal      = [100000 0];
obstacleR = 0;

%[最高速度[mm/s],最高回頭速度[rad/s],最高加減速度[mm/ss],最高加減回頭速度[rad/ss], 速度解像度[mm/s],回頭速度解像度[rad/s]]
Kinematic = [6000, (40.0)/180*pi, 1000, (40.0)/180*pi,100, (2)/180*pi];
%評価関数のパラメータ [heading,dist,velocity,predictDT]
evalParam = [0.1,0.45,0.1,4.0]; % [0.1,0.45,0.1,4.0]
area      = [track.xmin, track.xmax, track.ymin, track.xmax];
%============================================================

%---road data-----------------
roadx = 0:2000:100000;
road=[];
for i=1:length(roadx)
    road=[road;roadx(i) 3250];
    road=[road;roadx(i) -3250];    
end
%------------------------------

% RUN
% INITIALIZE SAVER
myinfo     = get_trackinfo(track, mycar.pos, othercars);
traj = init_traj(track, mycar, othercars);
while sim.flag && ishandle(fig)
    % KEYBOARD CONTROLLER
    switch key_pressed 
        case ''
        case {'leftarrow', 'semicolon'}
            %mycar.vel(2) = mycar.vel(2)+20;
             mycar.vel(2) = mycar.vel(2)+2;
        case {'rightarrow', 'quote'}
            %mycar.vel(2) = mycar.vel(2)-20;
            mycar.vel(2) = mycar.vel(2)-2;
        case {'uparrow', 'leftbracket'}
            %mycar.vel(1) = mycar.vel(1)+5000; % 10000 mm/s = 36 km/h
            mycar.vel(1) = mycar.vel(1)+200;
        case {'downarrow', 'slash'}
            %mycar.vel(1) = mycar.vel(1)-5000; % 10000 mm/s = 36 km/h
            mycar.vel(1) = mycar.vel(1)-200;
        case 'space'
            mycar.vel = [0 0];
        case {'1', '2', '3', '4', '5', '6'}
            nr_lane = str2num(key_pressed);
            mycar = set_mycar(mycar, get_posintrack(track, 1, 0, nr_lane, 0), [0 0]);
        case 's' % MAKE A CAR GO STRAIGHT
            mycar.pos(3) = 0; mycar.vel(2) = 0;
        case 'p'
            if isequal(sim.mode, 'RUN'), sim.mode = 'PAUSE'; 
            elseif isequal(sim.mode, 'PAUSE'), sim.mode = 'RUN'; end
        case 'q' 
            sim.mode = 'QUIT';
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
            
            othercars  = update_othercars(othercars, sim);
            
            %- pedestrian
            if pedestrians.n > 0
                pedestrians = respawn_pedestrians(pedestrians,track);
                pedestrians = update_pedestrians(pedestrians,mycar,sim);
            end

            if FLAG_DynamicWindow
                if pedestrians.n > 0
                    pos_human =[];
                    for i = 1:pedestrians.n
                        pos_human = [pos_human;pedestrians.person{i}.bd.circle];
                    end
                    obstacle  = [pos_human;road;othercars.car{1}.bd;othercars.car{2}.bd];
                    addobstacle = [(othercars.car{1}.bd(1,:)+othercars.car{1}.bd(2,:))*0.5;...
                                  (othercars.car{2}.bd(1,:)+othercars.car{2}.bd(2,:))*0.5];
                    obstacle = [obstacle;addobstacle];          
                else
                    obstacle  = [road;othercars.car{1}.bd;othercars.car{2}.bd];
                    addobstacle = [(othercars.car{1}.bd(1,:)+othercars.car{1}.bd(2,:))*0.5;...
                                  (othercars.car{2}.bd(1,:)+othercars.car{2}.bd(2,:))*0.5];
                    obstacle = [obstacle;addobstacle];
                end
                [mycar,traj_DWA, eval_DWA] = Control_by_DynamicWindowApproach(mycar,pedestrians, sim, goal,obstacle,obstacleR, Kinematic, evalParam, area, PLOT_DWA, FLAG_methodDWA);
            end
            
            mycar    = update_mycar2(mycar, sim, othercars, pedestrians);
            myinfo     = get_trackinfo(track, mycar.pos, othercars);
            ms_update  = etime(clock, clk_update)*1000;
            titlecol = 'w';
            
            % SAVE TRAJ
            traj = add_traj(traj, mycar, myinfo);
                        
            % TERMINATE CONDITIONS        
             if is_goal(mycar,myinfo,track,1)  % added by kumano
                fprintf(1, 'SUCCEEDED!! \n');
                mycar = init_mycar(get_posintrack(track, 1, 0, 1, 0),ini_vel); % mod by kumano
                pedestrians = reset_pedestrians(pedestrians, track);   % add by kumano
                othercars = reset_othercars2(othercars,track);
             end
            if is_insidetrack(myinfo) == 0
                fprintf(2, 'OUTSIDE THE TRACK. \n');
                mycar = init_mycar(get_posintrack(track, 1, 0, 1, 0),ini_vel); % mod by kumano
                pedestrians = reset_pedestrians(pedestrians, track);   % add by kumano
                othercars = reset_othercars2(othercars,track);                
            end
            %if is_carcrashed(myinfo)
            if is_carcrashed2(mycar) % mod by kumano
                fprintf(2, 'COLLISION OCCURRED. \n');
                mycar.vel = [0 0];
                mycar = init_mycar(get_posintrack(track, 1, 0, 1, 0),ini_vel); % mod by kumano
                pedestrians = reset_pedestrians(pedestrians, track);   % add by kumano
                othercars = reset_othercars2(othercars,track);
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
    PLOT_RFS_PEDESTRIAN  = 1; % 1
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
    axisinfo = plot_track(track, FILL_LANES);
    plot_axisinfo(axisinfo);
    plot_othercars(othercars, SIMPLECARSHAPE, REALCARSHAPE);
    %--plot pedestrians---- added by kumano---
    if pedestrians.n > 0
        plot_pedestrians(pedestrians, SIMPLECARSHAPE, REALCARSHAPE, PLOT_RFS_PEDESTRIAN,PLOT_COLLISION_AREA);
        if PLOT_VELOCITY_OBSTACLE
           plot_velocityObstacle(mycar, pedestrians);
        end
    end
    %-----------------------------------------
    plot_mycar(mycar, PLOT_FUTURE_CARPOSES, PLOT_CAR_PATHS, SIMPLECARSHAPE, REALCARSHAPE, PLOT_RFS);
    %plot_traj(traj);
    %----
    %plot_title(titlestr, titlecol, titlefontsize);
    drawnow;
    ms_plot = etime(clock, clk_plot)*1000;
    
end
fprintf(2, 'SIMULATION TERMINATED \n');

%%
