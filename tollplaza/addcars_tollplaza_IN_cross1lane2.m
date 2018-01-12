function othercars = addcars_tollplaza_IN_cross1lane2(othercars, track, nr_cars)
% ADD CARS IN RANDOM POSTIONS
% VERBOSE = 0;



for carid = 1:nr_cars
    
    laneidx= 1;
    tmppos = get_posintrack(track, 1, 0, laneidx, 0);
    x = (40-(carid-1)*10)*10^3;
    y = tmppos(2);
    carpos = [x y 0];
    othercars.car{carid}.goallane = 13;
    
    othercars.car{carid}.flgPlaza = 0; % 0:on straight lane ,1:on plaza lane
    othercars.car{carid}.flgIDM = 0; % 0:before controlling by IDM ,1:velocity control by IDM (both are on condition in the plaza)
    othercars.car{carid}.angry = 0;
    segidx = 1;
    othercars = add_othercars(othercars, carpos, [17500 0], 'normal',1, laneidx, segidx); % 10000 mm/s = 36 km/h
end

end


function othercars = add_othercars(othercars, pos, vel, ctrlmode, track_idx,lane_idx,seg_idx)
if nargin == 3
    ctrlmode = 'normal';
end
othercars.n = othercars.n + 1;
othercars.car{othercars.n}.pos = pos;
othercars.car{othercars.n}.vel = vel;
othercars.car{othercars.n}.ctrlmode = ctrlmode;
othercars.car{othercars.n}.paths = [];

othercars.car{othercars.n}.save.pos = pos; 
othercars.car{othercars.n}.save.vel = vel;
othercars.car{othercars.n}.save.track_idx = track_idx;
othercars.car{othercars.n}.save.lane_idx  = lane_idx;
othercars.car{othercars.n}.save.seg_idx   = seg_idx;
end