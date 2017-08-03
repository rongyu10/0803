ccc
%% MANUALLY COLLECT DRIVING DEMONSTRATIONS
ccc;
addpath(genpath('./core'))
addpath(genpath('./pedestrian'))
%========== Simulator Settings ================================ 
% INITIALIZE ENVIRONMENT
TRACK_TYPE = 'SHORT_STRAIGHT';
NR_LANE    = 1;
LANE_WIDTH = 6500; % LANE WIDTH
track      = init_track(NR_LANE, LANE_WIDTH, TRACK_TYPE);
sim        = init_sim(0.05); % dt = 0.02 [sec]
othercars  = init_othercars();
nr_cars    = 0;  % 11
ini_vel    = [2000 0];
othercars  =  addcars_shoppingArea(othercars, track, nr_cars);
mycar      = init_mycar(get_posintrack(track, 1, 0, 1, 0),ini_vel);
%--pedestrians
nr_people = 6;   % Number of Pedestrians
pedestrians= init_pedestrians();
pedestrians=  addpedestrians(pedestrians, track, nr_people);

% INITIALIZE FIGURE
figsz     = [1 4 8 4]/10;
figtitle  = 'GUI-GUI CONTROL SIMULATOR in Shopping Area';
axespos   = [0.03, 0.02, 0.95, 0.84]; 
fig       = get_fig(figsz, figtitle, axespos);
set(gcf,'Color', [0.1, 0.25, 0.2] ); hold on;
ms_update = 0; ms_plot = 0;
%============================================================

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
             mycar.vel(2) = mycar.vel(2)+10;
        case {'rightarrow', 'quote'}
            %mycar.vel(2) = mycar.vel(2)-20;
            mycar.vel(2) = mycar.vel(2)-10;
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
            pedestrians = respawn_pedestrians(pedestrians,track);
            pedestrians = update_pedestrians(pedestrians,mycar,sim);
            

            mycar    = update_mycar2(mycar, sim, othercars, pedestrians);
            myinfo     = get_trackinfo(track, mycar.pos, othercars);
            ms_update  = etime(clock, clk_update)*1000;
            titlecol = 'w';
                        
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
    PLOT_FUTURE_CARPOSES = 1; % 1
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
    plot_pedestrians(pedestrians, SIMPLECARSHAPE, REALCARSHAPE, PLOT_RFS_PEDESTRIAN,[]);    
    %-----------------------------------------
    plot_mycar(mycar, PLOT_FUTURE_CARPOSES, PLOT_CAR_PATHS, SIMPLECARSHAPE, REALCARSHAPE, PLOT_RFS);
    %plot_traj(traj);
    %----
    plot_title(titlestr, titlecol, titlefontsize);
    drawnow;
    ms_plot = etime(clock, clk_plot)*1000;
    
end
fprintf(2, 'SIMULATION TERMINATED \n');

%%
