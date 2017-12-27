function pedestrians = addpedestrians(pedestrians, track, nr_people)
% ADD Pedestrians IN RANDOM POSTIONS
VERBOSE = 0;

%--- Setting Pedestrians-----
for iperson = 1:nr_people
    segidx  = randi(track.nr_seg-2)+2;
    laneidx = 1;
    seglen = track.seg{segidx}.d;
    randoffset = seglen*rand*0.5;
    pos = get_posintrack(track, segidx, randoffset, laneidx, 0);
    
    %-------
    %var_speed = 1000;
    %speed = 1500 + var_speed *(rand()- 0.5);  % randam seed
    speed = 700;
    offset_y     = (track.ymax - track.ymin)*(rand()-0.5);
    iflag = rand();
    if iflag > 0.5
        offset_angle = randi(60)-30;   
    elseif iflag <= 0.5
        offset_angle = randi(60)+150;
    else
        offset_angle = randi(359);
    end

    pedestrians = add_pedestrians(pedestrians, pos, [speed 0], 'normal');    
    pedestrians.person{iperson, 1}.pos(2) = pedestrians.person{iperson, 1}.pos(2) + offset_y;
    pedestrians.person{iperson, 1}.pos(3)   = offset_angle;
end

%{
%　歩行者の設定
%---1人目
iperson = 1;
laneidx = 1;
segidx  = 3;
seglen = track.seg{segidx}.d;
randoffset = seglen*rand*0.5;
pos = get_posintrack(track, segidx, randoffset, laneidx, 0);

pedestrians = add_pedestrians(pedestrians, pos, [3000 0], 'normal');
pedestrians.person{iperson, 1}.pos(2) = pedestrians.person{iperson, 1}.pos(2) +1000;
pedestrians.person{iperson, 1}.pos(3) = 15;

%---2人目
laneidx= 1;
segidx = 7;
seglen = track.seg{segidx}.d;
randoffset = seglen*rand*0.5;
pos = get_posintrack(track, segidx, randoffset, laneidx, 0);

pedestrians = add_pedestrians(pedestrians, pos, [3000 0], 'normal');
pedestrians.person{2, 1}.pos(2) = pedestrians.person{2, 1}.pos(2) -1000;
pedestrians.person{2, 1}.pos(3) = 135;

%---2人目
laneidx= 1;
segidx = 7;
seglen = track.seg{segidx}.d;
randoffset = seglen*rand*0.5;
pos = get_posintrack(track, segidx, randoffset, laneidx, 0);

pedestrians = add_pedestrians(pedestrians, pos, [3000 0], 'normal');
pedestrians.person{2, 1}.pos(2) = pedestrians.person{2, 1}.pos(2) -1000;
pedestrians.person{2, 1}.pos(3) = 135;
%}


end


function pedestrians = add_pedestrians(pedestrians, pos, vel, ctrlmode)
if nargin == 3
    ctrlmode = 'normal';
end
pedestrians.n = pedestrians.n + 1;
pedestrians.person{pedestrians.n}.pos = pos;
pedestrians.person{pedestrians.n}.vel = vel;
pedestrians.person{pedestrians.n}.ctrlmode = ctrlmode;
pedestrians.person{pedestrians.n}.paths = [];
pedestrians.person{pedestrians.n}.vel_save = vel;
pedestrians.person{pedestrians.n}.pos_save = pos;
end
