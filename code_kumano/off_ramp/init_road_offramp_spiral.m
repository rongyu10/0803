function road  = init_road_offramp_spiral(varargin)

%========== Simulator Settings ================================ 
    % INITIALIZE ENVIRONMENT
    NR_TRACK      = 23;
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
    LANE_WIDTH = 5*10^3; % LANE WIDTH IS FIXED TO 3.5M
    NR_SEG     = 1;
    LANE_LENGTH= 20*10^3;
    ctlPt = [20 5; 15 5; 10 2.5;5 0.5; 0 0]; % [mm]
    n_t   = 25; % number of output data
    P = bezierCurve(ctlPt,n_t);
    p1 = [0 0];
    p2 = [20 0];
    p3 = [20 5];
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
    LANE_WIDTH = 5*10^3; % LANE WIDTH IS FIXED TO 3.5M
    NR_SEG     = 4;
    LANE_LENGTH= 80*10^3;
    %-- start position---
    startpos = [260 13.0 0]*10^3;
    %--------------------
    road.track{3} = add_track_tollplaza(road.track{2}, NR_LANE, LANE_WIDTH, NR_SEG, LANE_LENGTH, [], TRACK_TYPE,startpos);
    %--------------
    
    %------ Spiral -----------------------------
    R      =  50*10^3;
    diff_R =  2.5*10^3;
    %--
    TRACK_TYPE = 'LEFT_TURN';
    NR_LANE    = 1;
    LANE_WIDTH = 5*10^3; % LANE WIDTH IS FIXED TO 3.5M
    NR_SEG     = 1;
    LANE_LENGTH= R;
    
    for i = 4:NR_TRACK 
       road.track{i} = add_track_tollplaza(road.track{i-1}, NR_LANE, LANE_WIDTH, NR_SEG, LANE_LENGTH, [], TRACK_TYPE);
       R = R - diff_R; 
       LANE_LENGTH = R;
    end    
    %---------------------------------------------
    
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