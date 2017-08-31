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



% INITIALIZE FIGURE
figsz     = [1 4 8 4]/10;
figtitle  = 'GUI-GUI CONTROL SIMULATOR at TOLL PLAZA';
axespos   = [0.03, 0.02, 0.95, 0.84]; 
fig       = get_fig(figsz, figtitle, axespos);
set(gcf,'Color', [0.1, 0.25, 0.2] ); hold on;


FILL_LANES = 1;
    axisinfo = plot_track_tollplaza(road, FILL_LANES);
    plot_axisinfo_tollplaza(axisinfo);
    
    drawnow;



fprintf(2, 'SIMULATION TERMINATED \n');

%%