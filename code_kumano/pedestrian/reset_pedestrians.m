function pedestrians = reset_pedestrians(pedestrians, track)
% ADD Pedestrians IN RANDOM POSTIONS
VERBOSE = 0;

%--- Setting Pedestrians-----
for iperson = 1:pedestrians.n
    x = pedestrians.person{iperson, 1}.pos(1);
    if x < 10000
        segidx  = randi(track.nr_seg-2)+2;
        laneidx = 1;
        seglen = track.seg{segidx}.d;
        randoffset = seglen*rand*0.5;
        pos = get_posintrack(track, segidx, randoffset, laneidx, 0);
        %-------
        %speed = 2000 + (500 *rand()- 500);  % randam soeed
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
        pedestrians.person{iperson, 1}.vel(1) = speed;
        pedestrians.person{iperson, 1}.pos    = pos;
        pedestrians.person{iperson, 1}.pos(2) = pedestrians.person{iperson, 1}.pos(2) + offset_y;
        pedestrians.person{iperson, 1}.pos(3) = offset_angle;
    end
end

end
