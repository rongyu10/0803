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

figsz     = [1 4 8 4]/10;
figtitle  = 'GUI-GUI CONTROL SIMULATOR at TOLL PLAZA';
axespos   = [0.03, 0.02, 0.95, 0.84]; 
fig       = get_fig(figsz, figtitle, axespos);
set(gcf,'Color', [0.1, 0.25, 0.2] ); hold on;
ishandle(fig);

FILL_LANES           = 1;
axisinfo = plot_track_tollplaza(road, FILL_LANES);
plot_axisinfo_tollplaza(axisinfo);

% MAKE LANECHANGE PATH
dx = 175*10^3/3; % x-cood.interval of control points for bezier curve
for i = 1:7:15 % i:start lane j:goal lane
    for j = 1:3
        ctlPt = [100*10^3 12250 - 3500 * j; 100*10^3 + dx 12250 - 3500 * j; 100*10^3 + 2*dx 77.5*10^3 - 5*10^3*i; 100*10^3 + 3*dx 77.5*10^3 - 5*10^3*i];
        [laneChangePath{i,j}, lengthP{i,j}] = bezierCurve(ctlPt);
        plot(laneChangePath{i,j}(:,1),laneChangePath{i,j}(:,2), ':', 'Color', [1 1 1], 'LineWidth', 1); 
        
    end
end
hold on
% ctlPt = [100*10^3 12250 - 3500 * 1; 100*10^3 + dx 12250 - 3500 * 1; 100*10^3 + 2*dx 77.5*10^3 - 5*10^3*1; 100*10^3 + 3*dx 77.5*10^3 - 5*10^3*1];
%  [Path, Length] = bezierCurve(ctlPt);
% plot(Path(:,1),Length(:,2), ':', 'Color', [0 1 1], 'LineWidth', 1); 

drawnow;

%%