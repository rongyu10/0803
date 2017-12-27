function road  = init_road_offramp(varargin)

%========== Simulator Settings ================================ 
    % INITIALIZE ENVIRONMENT
    NR_TRACK      = 5;
    road.track    = cell(NR_TRACK, 1);
    road.nr_track = NR_TRACK;
    %----------------------------------
    TRACK_TYPE = 'STRAIGHT';
    NR_LANE    = 3;
    LANE_WIDTH = 3.5*10^3; % LANE WIDTH IS FIXED TO 3.5M
    NR_SEG     = 22;
    LANE_LENGTH= 440*10^3;
    road.track{1} = add_track_tollplaza([],NR_LANE, LANE_WIDTH, NR_SEG, LANE_LENGTH, [], TRACK_TYPE);
    %----------------------------------
    TRACK_TYPE = 'PLAZA';
    NR_LANE    = 1;
    LANE_WIDTH = 4.5*10^3; % LANE WIDTH IS FIXED TO 3.5M
    NR_SEG     = 1;
    LANE_LENGTH= 20*10^3;
    ctlPt = [20 4.5; 15 4.5; 10 2.25; 5 1.0; 0 0]; % [mm]
    n_t   = 25; % number of output data
    P = bezierCurve(ctlPt,n_t);
    p1 = [0 0];
    p2 = [20 0];
    p3 = [20 4.5];
    p4 = [0 0];
    pointdata = [p1;p2;p3;P;p4;p1]*10^3;
    %-- start position---
    startpos = [240 10.5 0]*10^3;
    %--------------------
    road.track{2} = add_track_tollplaza(road.track{1}, NR_LANE, LANE_WIDTH, NR_SEG, LANE_LENGTH, pointdata, TRACK_TYPE,startpos);
    %----------------------------------
    
    %--------------
    TRACK_TYPE = 'STRAIGHT';
    NR_LANE    = 1;
    LANE_WIDTH = 4.5*10^3; % LANE WIDTH IS FIXED TO 3.5M
    NR_SEG     = 4;
    LANE_LENGTH= 80*10^3;
    %-- start position---
    startpos = [260 12.75 0]*10^3;
    %--------------------
    road.track{3} = add_track_tollplaza(road.track{2}, NR_LANE, LANE_WIDTH, NR_SEG, LANE_LENGTH, [], TRACK_TYPE,startpos);
    %--------------
    
    %-----------------------------------
    %TRACK_TYPE = 'LEFT_TURN';
    TRACK_TYPE = 'LEFT_TURN45';
    NR_LANE    = 1;
    LANE_WIDTH = 4.5*10^3; % LANE WIDTH IS FIXED TO 3.5M
    NR_SEG     = 1;
    LANE_LENGTH= 60*10^3;
    road.track{4} = add_track_tollplaza(road.track{3}, NR_LANE, LANE_WIDTH, NR_SEG, LANE_LENGTH, [], TRACK_TYPE);

    TRACK_TYPE = 'STRAIGHT';
    NR_LANE    = 1;
    LANE_WIDTH = 4.5*10^3; % LANE WIDTH IS FIXED TO 3.5M
    NR_SEG     = 1;
    LANE_LENGTH= 15*10^3;
    road.track{5} = add_track_tollplaza(road.track{4}, NR_LANE, LANE_WIDTH, NR_SEG, LANE_LENGTH, [], TRACK_TYPE);
    %-----------------------------------
    
tmp_xmin =[];
tmp_xmax =[];
tmp_ymin =[];
tmp_ymax =[];
for i= 1:NR_TRACK
   tmp_xmin =[tmp_xmin,road.track{i}.xmin];
   tmp_xmax =[tmp_xmax,road.track{i}.xmax];
   tmp_ymin =[tmp_ymin,road.track{i}.ymin];
   tmp_ymax =[tmp_ymax,road.track{i}.ymax];
end
road.xmin = min(tmp_xmin);
road.xmax = max(tmp_xmax);
road.ymin = min(tmp_ymin);
road.ymax = max(tmp_ymax);
%----------------------------------
%======================================================================


end