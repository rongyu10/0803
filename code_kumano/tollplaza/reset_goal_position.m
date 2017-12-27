function pos_goal = reset_goal_position(track, idx_goalgate, mycar, othercars)
persistent first_flag h
if isempty(first_flag)
    first_flag = true;
end

%-----Goal Gate Position -----
nr_lane = track.nr_lane;
width   = track.width;

ps      = track.seg{1}.p;
interval_Y = width/nr_lane;
dY  = [0 interval_Y];

i  = idx_goalgate;
p1 = ps(4, :) - dY*(i-1);
p2 = ps(4, :) - dY*i;
p3 = ps(3, :) - dY*i;
p4 = ps(3, :) - dY*(i-1);
%bd = [p1;p2;p3;p4;p1];
pos_goalgate = (p1 + p3)*0.5;
%------------------------------

if (mycar.pos(1) < 100*10^3) % in initial straight
   pos_goal = pos_goalgate;
else
    %-- offset position from the goal gate (may need to be modified)
    pos_goalgate_offset    = pos_goalgate;
    pos_goalgate_offset(1) = pos_goalgate_offset(1) - 35*10^3;
    %---------------
    pos_mycar         = mycar.pos(1:2);
    
    %---vector from mycar to the goal gate-----------
    %vec_mycar_to_gate = pos_goalgate - pos_mycar;
    vec_mycar_to_gate = pos_goalgate_offset - pos_mycar;
    %------------------------------------------------
    searchbd = searchArea_RectType(pos_mycar,vec_mycar_to_gate); % get searching box to find a car of end of the line.
    pos_othercars = get_allPosition(othercars);
    P = inpolygon(pos_othercars(:,1), pos_othercars(:,2), searchbd(:, 1), searchbd(:, 2)); % find cars inside the searching box
    
    if sum(P)==0  % no car in the searching box
       pos_goal = pos_goalgate;
    else
       tmp_pos_othercars = pos_othercars(P,:);
       [~,idx_min] = min(tmp_pos_othercars(:,1)); % get car ID with minmum X coordinate
       pos_goal = tmp_pos_othercars(idx_min,:);
    end
    
    %-- to diplay the searching box----
    if first_flag
       h.bd = plot(searchbd(:,1),searchbd(:,2),'-','Color','b','LineWidth',2);
       first_flag = false; 
    else
       h.bd.XData = searchbd(:,1)';
       h.bd.YData = searchbd(:,2)'; 
    end 
    %---------------------------------
end

end

%----------------------------------------------------------------------
function bd = searchArea_RectType(pos,vec)

%--Area Setting------
W = 15*10^3; % Width of searching box
%--------------------

length = norm(vec);
theta  = atan2(vec(2),vec(1));

tmp_bd  = [0 -W/2; length -W/2; length  W/2; 0  W/2; 0 -W/2];
%tmp_bd  = [0 -W/4; length -W/4; length  W*(3/4); 0  W*(3/4); 0 -W/4];
%tmp_bd  = [0 0; length 0; length  W; 0  W; 0 0];
%--rotation------
c = cos(theta);
s = sin(theta);
rotaion = [c -s;s c]';
tmp_bd = tmp_bd*rotaion;
%----------------
%--translation---
nsize = size(tmp_bd,1);
bd = tmp_bd + repmat(pos(1:2),nsize,1);
%----------------

end

function allpos = get_allPosition(othercars)

nr_cars = othercars.n;
allpos  = zeros(nr_cars,2);

for i=1:nr_cars
    allpos(i,1:2) = othercars.car{i}.pos(1:2);
end

end

%----------------------------------------------------------------------