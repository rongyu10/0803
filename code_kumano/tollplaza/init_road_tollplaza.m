function road      = init_road_tollplaza(varargin)

%========== Simulator Settings ================================ 
if (nargin == 0)||(strcmp(varargin{1},'enter')) % EnterSide Road
    % INITIALIZE ENVIRONMENT
    NR_TRACK      = 4;
    road.track    = cell(NR_TRACK, 1);
    road.nr_track = NR_TRACK;
    %----------------------------------
    TRACK_TYPE = 'STRAIGHT';
    NR_LANE    = 3;
    LANE_WIDTH = 3.5*10^3; % LANE WIDTH IS FIXED TO 3.5M
    NR_SEG     = 10;
    LANE_LENGTH= 100*10^3;
    road.track{1} = add_track_tollplaza([],NR_LANE, LANE_WIDTH, NR_SEG, LANE_LENGTH, [], TRACK_TYPE);
    %--------------
    TRACK_TYPE = 'PLAZA';
    NR_LANE    = 1;
    LANE_WIDTH = 75*10^3; % LANE WIDTH IS FIXED TO 3.5M
    NR_SEG     = 1;
    LANE_LENGTH= 175*10^3;
    ctlPt = [175 75; 125 75; 75 45 ;25 10.5; 0 10.5]; % [mm]
    P = bezierCurve(ctlPt);
    p1 = [0 0];
    p2 = [175 0];
    p3 = [175 75];
    p4 = [0 10.5];
    pointdata = [p1;p2;p3;P;p4;p1]*10^3;
    road.track{2} = add_track_tollplaza(road.track{1}, NR_LANE, LANE_WIDTH, NR_SEG, LANE_LENGTH, pointdata, TRACK_TYPE);
    %--------------
    TRACK_TYPE = 'STRAIGHT';
    NR_LANE    = 15;
    LANE_WIDTH = 5*10^3; % LANE WIDTH IS FIXED TO 3.5M
    NR_SEG     = 1;
    LANE_LENGTH= 30*10^3;
    road.track{3} = add_track_tollplaza(road.track{2}, NR_LANE, LANE_WIDTH, NR_SEG, LANE_LENGTH, [], TRACK_TYPE);
    %--------------
    TRACK_TYPE = 'STRAIGHT';
    NR_LANE    = 15;
    LANE_WIDTH = 5*10^3; % LANE WIDTH IS FIXED TO 3.5M
    NR_SEG     = 1;
    LANE_LENGTH= 15*10^3;
    road.track{4} = add_track_tollplaza(road.track{3}, NR_LANE, LANE_WIDTH, NR_SEG, LANE_LENGTH, [], TRACK_TYPE);
    
elseif (strcmp(varargin{1},'exit')) % ExitSide Road
    % INITIALIZE ENVIRONMENT
    NR_TRACK      = 5;
    road.track    = cell(NR_TRACK, 1);
    road.nr_track = NR_TRACK;
    %--------------
    TRACK_TYPE = 'STRAIGHT';
    NR_LANE    = 15;
    LANE_WIDTH = 5*10^3; % LANE WIDTH IS FIXED TO 3.5M
    NR_SEG     = 1;
    LANE_LENGTH= 15*10^3;
    road.track{1} = add_track_tollplaza([], NR_LANE, LANE_WIDTH, NR_SEG, LANE_LENGTH, [], TRACK_TYPE);    
    %--------------
    TRACK_TYPE = 'STRAIGHT';
    NR_LANE    = 15;
    LANE_WIDTH = 5*10^3; % LANE WIDTH IS FIXED TO 3.5M
    NR_SEG     = 1;
    LANE_LENGTH= 30*10^3;
    road.track{2} = add_track_tollplaza(road.track{1}, NR_LANE, LANE_WIDTH, NR_SEG, LANE_LENGTH, [], TRACK_TYPE);
    %--------------
    TRACK_TYPE = 'PLAZA';
    NR_LANE    = 1;
    LANE_WIDTH = 75*10^3; % LANE WIDTH IS FIXED TO 3.5M
    NR_SEG     = 1;
    LANE_LENGTH= 240*10^3;
    ctlPt = [240 10.5; 215 10.5; 140 45; 40 75; 0 75]; %

    P = bezierCurve(ctlPt);
    p1 = [0   0];
    p2 = [240 0];
    p3 = [240 10.5];
    p4 = [0   75];
    pointdata = [p1;p2;p3;P;p4;p1]*10^3;
    road.track{3} = add_track_tollplaza(road.track{2}, NR_LANE, LANE_WIDTH, NR_SEG, LANE_LENGTH, pointdata, TRACK_TYPE);    
    %----------------------------------
    TRACK_TYPE = 'STRAIGHT';
    NR_LANE    = 3;
    LANE_WIDTH = 3.5*10^3; % LANE WIDTH IS FIXED TO 3.5M
    NR_SEG     = 9;
    LANE_LENGTH= 90*10^3;
    road.track{4} = add_track_tollplaza(road.track{3},NR_LANE, LANE_WIDTH, NR_SEG, LANE_LENGTH, [], TRACK_TYPE);
    %----------------------------------
    TRACK_TYPE = 'STRAIGHT';
    NR_LANE    = 3;
    LANE_WIDTH = 3.5*10^3; % LANE WIDTH IS FIXED TO 3.5M
    NR_SEG     = 1;
    LANE_LENGTH= 10*10^3;
    road.track{5} = add_track_tollplaza(road.track{4},NR_LANE, LANE_WIDTH, NR_SEG, LANE_LENGTH, [], TRACK_TYPE);
    
end

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