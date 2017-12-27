function pos_goalgate = set_goalgate_position(track, idx_goalgate)


nr_lane = track.nr_lane;
width   = track.width;

ps      = track.seg{1}.p;
interval_Y = width/nr_lane;
dY  = [0 interval_Y];

i = idx_goalgate;
p1 = ps(4, :) - dY*(i-1);
p2 = ps(4, :) - dY*i;
p3 = ps(3, :) - dY*i;
p4 = ps(3, :) - dY*(i-1);
%bd = [p1;p2;p3;p4;p1];
pos_goalgate = (p1 + p3)*0.5;


end