function othercars = addcars_tollplaza_IN_cross2lane(othercars, track, nr_cars, laneChangePath)
% ADD CARS IN RANDOM POSTIONS
% VERBOSE = 0;



for carid = 1:nr_cars
    
    
    if carid <= 30
        laneidx= 1;
        tmppos = get_posintrack(track, 1, 0, laneidx, 0);
        x = (240-(carid-1)*40)*10^3;
        y = tmppos(2);
        othercars.car{carid}.goallane = 13;
    else
        laneidx= 2;
        tmppos = get_posintrack(track, 1, 0, laneidx, 0);
        x = (220-(carid-30-1)*40)*10^3;
        y = tmppos(2);
        othercars.car{carid}.goallane = 14;
    end
        
    othercars.car{carid}.time_TTC = 0.0;
    othercars.car{carid}.pathTranslated = laneChangePath{othercars.car{carid}.goallane, laneidx};
    
    if x < track.xmax
        othercars.car{carid}.flgPlaza = 0; % 0:on straight lane ,1:on plaza lane
        carpos = [x y 0];
    else
        othercars.car{carid}.flgPlaza = 1; % 0:on straight lane ,1:on plaza lane
        [~,idx]=min(abs(othercars.car{carid}.pathTranslated(:,1) - x));
        
        nData = size(othercars.car{carid}.pathTranslated,1);
        
        if idx~=nData
            vx= othercars.car{carid}.pathTranslated(idx+1,1)-othercars.car{carid}.pathTranslated(idx,1);
            vy= othercars.car{carid}.pathTranslated(idx+1,2)-othercars.car{carid}.pathTranslated(idx,2);
        else
            vx= othercars.car{carid}.pathTranslated(idx,1)-othercars.car{carid}.pathTranslated(idx-1,1);
            vy= othercars.car{carid}.pathTranslated(idx,2)-othercars.car{carid}.pathTranslated(idx-1,2);
        end
        
        carpos = [othercars.car{carid}.pathTranslated(idx,1) othercars.car{carid}.pathTranslated(idx,2) atan(vy/vx)*180/pi];
    end
    
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