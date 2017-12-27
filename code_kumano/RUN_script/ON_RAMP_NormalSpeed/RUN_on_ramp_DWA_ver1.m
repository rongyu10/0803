%ccc
%% MANUALLY COLLECT DRIVING DEMONSTRATIONS
%ccc;
warning('off','all')
addpath(genpath('../../core'))
addpath(genpath('../../on_ramp'))
addpath(genpath('../../bezier'))
addpath(genpath('../../intelligentDriverModel'))
addpath(genpath('../../diverseDriverSetting'))
addpath(genpath('../../dynamicWindow'))
addpath(genpath('../../turnSignal'))
ccc;
%========== Simulator Settings ================================ 
% INITIALIZE ENVIRONMENT
TRACK_TYPE = 'LONG_STRAIGHT';
NR_LANE    = 2;
LANE_WIDTH = 3500; % LANE WIDTH IS FIXED TO 3.5M
track      = init_track(NR_LANE, LANE_WIDTH, TRACK_TYPE);
sim        = init_sim(0.07); % dt = 0.05 [sec]
othercars  = init_othercars();
desired_velocity = 15000;
ratio_polite_driver = 0.3;
othercars  = init_othercars_IDM(othercars,desired_velocity,ratio_polite_driver);
%nr_cars    = randi([++1 4]);
nr_cars    = 11;  % 11
othercars  = addcars2_onRamp(othercars, track, nr_cars);
ini_vel    = [5000 0];
mycar      = init_mycar(get_posintrack(track, 2, 0, 1, 0),ini_vel);

% INITIALIZE FIGURE
figsz     = [1 4 8 4]/10;
figtitle  = 'GUI-GUI CONTROL SIMULATOR';
axespos   = [0.03, 0.02, 0.95, 0.84]; 
fig       = get_fig(figsz, figtitle, axespos);
set(gcf,'Color', [0.1, 0.25, 0.2] ); hold on;
ms_update = 0; ms_plot = 0;

% LANE CHANGE SETTING--------
FLAG_LANECHANGE = 0;
dx = 15000/3; % x-cood.interval of control points for bezier curve
ctlPt = [0 0; dx 0; 2*dx -LANE_WIDTH; 3*dx -LANE_WIDTH];
[laneChangePath, lengthP] = bezierCurve(ctlPt);
ratioSpeed = lengthP/15000*1.1;
%----------------------------
%-- FLAG INTERACTION(Intelligent Driver Model) ---------
FLAG_INTERACTION = true;
%-------------------------------------------------------
%============================================================
%======= Dynamic Window Approach ============================
FLAG_DynamicWindow     = true;
FLAG_methodDWA         = 1;   % 1: normal DWA, 2: time-varying DWA, 3: DWA with velocity obstacles(under constraction) 
PLOT_DWA               = true;
PLOT_COLLISION_AREA    = false;
PLOT_VELOCITY_OBSTACLE = true;
goal      = [220000 -2000];
obstacleR = 0;

%[最高速度[mm/s],最高回頭速度[rad/s],最高加減速度[mm/ss],最高加減回頭速度[rad/ss], 速度解像度[mm/s],回頭速度解像度[rad/s]]
Kinematic = [20000, (40.0)/180*pi, 5000, (40.0)/180*pi,200, (1)/180*pi];
%評価関数のパラメータ [heading,dist,velocity,predictDT]
evalParam = [5.0,50.0,20.0,3.5]; % [0.1,0.45,0.1,4.0]
area      = [track.xmin, track.xmax, track.ymin, track.xmax];
%============================================================
%---road data-----------------
roadx = 0:500:250000;
road=[];
for i=1:length(roadx)
    road=[road;roadx(i) 3500];        
    road=[road;roadx(i) 2000];    
    road=[road;roadx(i) -5000];    
end

roadx2 = 190000:500:250000;
for  i=1:length(roadx2)
    road=[road;roadx2(i) 0];    
end

%-----------------------------

% RUN
% INITIALIZE SAVER
myinfo = get_trackinfo(track, mycar.pos, othercars);
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
            mycar.vel(1) = mycar.vel(1)+1000;
        case {'downarrow', 'slash'}
            %mycar.vel(1) = mycar.vel(1)-5000; % 10000 mm/s = 36 km/h
            mycar.vel(1) = mycar.vel(1)-1000;
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
        case 'l'
            % LANE CHANGING (add by kumano)
            FLAG_LANECHANGE = 1;
        case 'x'
            % Checking for Intelligetn Driver Model (add by kumano)
            othercars.car{1}.vel(1) = othercars.car{1}.vel(1) - 2000;
        case 'c'
            % Checking for Intelligetn Driver Model (add by kumano)
            idxcar = ceil(othercars.n/2);
            othercars.car{idxcar}.vel(1) = othercars.car{idxcar}.vel(1) - 2000;
        case 'v'
            % Checking for Intelligetn Driver Model (add by kumano)
            idxcar = ceil((othercars.n)/2);
            othercars.car{(othercars.n)-1}.vel(1) = othercars.car{(othercars.n)-1}.vel(1) - 2000;
        case 'z'
            % Checking for Intelligetn Driver Model (add by kumano)
            [~,idx_rear] = get_neighboursCars(mycar, othercars);
            if idx_rear~=0
                othercars.car{idx_rear}.vel(1) = othercars.car{idx_rear}.vel(1) - 2000;
            end
        case 'b'
            % turn Signal
            mycar.turnSignal = 'left';
        case 'n'
            % turn Signal
            mycar.turnSignal = '';
        case 'm'
            % turn Signal
            mycar.turnSignal = 'right';              
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
            othercars  = respawn_othercars(othercars,track); % added by kumano
            
            % Intelligent Driver Model
            if FLAG_INTERACTION
              %othercars  = intelligentDriverModel_tmp(othercars,mycar,myinfo,track,sim);
              othercars  = ACCModel_tmp(othercars,mycar,myinfo,track,sim);
            end
            
            othercars  = update_othercars(othercars, sim);
            
            if FLAG_DynamicWindow
                  obstacle  =[];
                  for i=1:othercars.n
                     bd = get_carLargeBD(othercars.car{i}.pos);
                     obstacle = [obstacle;bd];
                  end
                  all_obstacle = [road;obstacle];
                  
                  % plot obstacles
%                   if sim.tick==2
%                     f = scatter(all_obstacle(:,1),all_obstacle(:,2));
%                   elseif sim.tick>2
%                     f.XData = all_obstacle(:,1);
%                     f.YData = all_obstacle(:,2);
%                   end
                    
                [mycar,traj_DWA, eval_DWA] = Control_by_DynamicWindowApproach(mycar,[], sim, goal,all_obstacle,obstacleR, Kinematic, evalParam, area, PLOT_DWA, FLAG_methodDWA);
            end
            
            mycar    = update_mycar(mycar, sim, othercars);
            myinfo     = get_trackinfo(track, mycar.pos, othercars);
            ms_update  = etime(clock, clk_update)*1000;
            titlecol = 'w';
        
            
            % SAVE TRAJ
            %traj = add_traj(traj, mycar, myinfo);
 
            % TERMINATE CONDITIONS
             if is_goal(mycar,myinfo,track,2)  % added by kumano
                fprintf(1, 'SUCCEEDED!! \n');
                idx_seg = randi(12);
                mycar = init_mycar(get_posintrack(track, idx_seg, 0, 1, 0),ini_vel); % mod by kumano
                myinfo     = get_trackinfo(track, mycar.pos, othercars);
                FLAG_LANECHANGE = 0;
             end
            if is_insidetrack(myinfo) == 0
                fprintf(2, 'OUTSIDE THE TRACK. \n');
                idx_seg = randi(12);
                mycar = init_mycar(get_posintrack(track, idx_seg, 0, 1, 0),ini_vel); % mod by kumano
                myinfo     = get_trackinfo(track, mycar.pos, othercars);
                FLAG_LANECHANGE = 0;
            end
            %if is_carcrashed(myinfo)
            if is_carcrashed2(mycar) % mod by kumano
                fprintf(2, 'COLLISION OCCURRED. \n');
                idx_seg = randi(12);
                mycar = init_mycar(get_posintrack(track, idx_seg, 0, 1, 0),ini_vel); % mod by kumano
                myinfo= get_trackinfo(track, mycar.pos, othercars);
                FLAG_LANECHANGE = 0;
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
    plot_mycar(mycar, PLOT_FUTURE_CARPOSES, PLOT_CAR_PATHS, SIMPLECARSHAPE, REALCARSHAPE, PLOT_RFS);
    %plot_traj(traj);
    %----
    plot_turnSignal(mycar,sim);
    %plot_laneChangePath(laneChangePathTranslated,FLAG_LANECHANGE); % add by kumano
    %----
    plot_title(titlestr, titlecol, titlefontsize);
    drawnow;
    ms_plot = etime(clock, clk_plot)*1000;
    
end
fprintf(2, 'SIMULATION TERMINATED \n');

%%
