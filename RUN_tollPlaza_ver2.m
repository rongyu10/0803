 ccc
%% MANUALLY COLLECT DRIVING DEMONSTRATIONS
ccc;
addpath(genpath('./core'))
addpath(genpath('./tollplaza'))
addpath(genpath('./bezier'))
addpath(genpath('./intelligentDriverModel'))
%--- set simulation
road      = init_road_tollplaza2();
sim       = init_sim(0.02); % dt = 0.02 [sec]
%------------------
%--- set othercars
othercars  = init_othercars();
nr_cars    = 20;  % 11
othercars.npl = 10;
othercars  = addcars2_tollplaza(othercars, road.track{1}, nr_cars);
%---------------
%--- set mycar--
ini_vel    = [20000 0];
mycar      = init_mycar(get_posintrack(road.track{1}, 1, 0, 2, 0),ini_vel);
myinfo     = get_trackinfo_tollplaza(road, mycar.pos, othercars);
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
FLAG_LANECHANGE  = false;
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
             mycar.vel(2) = mycar.vel(2)+10;
        case {'rightarrow', 'quote'}
            %mycar.vel(2) = mycar.vel(2)-20;
            mycar.vel(2) = mycar.vel(2)-10;
        case {'uparrow', 'leftbracket'}
            %mycar.vel(1) = mycar.vel(1)+5000; % 10000 mm/s = 36 km/h
            mycar.vel(1) = mycar.vel(1)+2500;
        case {'downarrow', 'slash'}
            %mycar.vel(1) = mycar.vel(1)-5000; % 10000 mm/s = 36 km/h
            mycar.vel(1) = mycar.vel(1)-2500;
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
            
            othercars  = respawn_othercars_tollplaza(othercars,road,sim); % added by kumano
            % Intelligent Driver Model
            othercars = intelligentDriverModel2(othercars,mycar,myinfo,road,sim);
            othercars  = update_othercars(othercars, sim);
            
            if FLAG_LANECHANGE ==1  % added by kumano
              mycar    = update_control_mycar(mycar, sim, othercars,laneChangePathTranslated,ratioSpeed);              
            else
              mycar    = update_mycar(mycar, sim, othercars);
            end

            myinfo     = get_trackinfo_tollplaza(road, mycar.pos, othercars);
            ms_update  = etime(clock, clk_update)*1000;
            titlecol = 'w';
            
            % SAVE TRAJ
            %traj = add_traj(traj, mycar, myinfo);
                        
            % TERMINATE CONDITIONS        
             if is_goal(mycar,myinfo,road.track{3},2)  % added by kumano
                fprintf(1, 'SUCCEEDED!! \n');
                mycar = init_mycar(get_posintrack(road.track{1}, 1, 0, 2, 0),ini_vel); % mod by kumano
                FLAG_LANECHANGE = false;
             end
            if is_insidetrack(myinfo) == 0
                fprintf(2, 'OUTSIDE THE TRACK. \n');
                mycar = init_mycar(get_posintrack(road.track{1}, 1, 0, 2, 0),ini_vel); % mod by kumano
                FLAG_LANECHANGE = false;
            end
            %if is_carcrashed(myinfo)
            if is_carcrashed2(mycar) % mod by kumano
                fprintf(2, 'COLLISION OCCURRED. \n');
                mycar = init_mycar(get_posintrack(road.track{1}, 1, 0, 2, 0),ini_vel); % mod by kumano
                FLAG_LANECHANGE = false;
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
    plot_axisinfo_tollplaza(axisinfo);
    plot_othercars(othercars, SIMPLECARSHAPE, REALCARSHAPE);
    plot_mycar(mycar, PLOT_FUTURE_CARPOSES, PLOT_CAR_PATHS, SIMPLECARSHAPE, REALCARSHAPE, PLOT_RFS);
    %plot_traj(traj);
    %----
    plot_title(titlestr, titlecol, titlefontsize);
    drawnow;
    ms_plot = etime(clock, clk_plot)*1000;
    
end
fprintf(2, 'SIMULATION TERMINATED \n');

%%