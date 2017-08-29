function othercars = addcars_tollplaza_OUT(othercars, track, nr_cars)
% ADD CARS IN RANDOM POSTIONS
VERBOSE = 0;

tolllane_section = 1;

for carid = 1:nr_cars
%     if carid <= othercars.npl
%        laneidx= 6;
%        tmppos = get_posintrack(track, 1, 0, laneidx, 0);
%        x = (3-carid)*20*10^3;
%        y = tmppos(2);
%        carpos = [x y 0];
%        othercars.car{carid}.goallane = 1;
%     elseif carid <= 2 * othercars.npl
%        laneidx= 7;
%        tmppos = get_posintrack(track, 1, 0, laneidx, 0);
%        x = (3-carid+othercars.npl)*20*10^3;
%        y = tmppos(2);
%        carpos = [x y 0];
%        othercars.car{carid}.goallane = 1;
%     elseif carid <= 3 * othercars.npl
%        laneidx= 9;
%        tmppos = get_posintrack(track, 1, 0, laneidx, 0);
%        x = (3-carid+othercars.npl*2)*20*10^3;
%        y = tmppos(2);
%        carpos = [x y 0];
%        othercars.car{carid}.goallane = 3;
%     else
%        laneidx= 10;
%        tmppos = get_posintrack(track, 1, 0, laneidx, 0);
%        x = (3-carid+othercars.npl*3)*20*10^3;
%        y = tmppos(2);
%        carpos = [x y 0];
%        othercars.car{carid}.goallane = 3;
%     end

    laneidx= 5 * (tolllane_section - 1) + randi(5);
    
    tolllane_section = tolllane_section + 1;
    if tolllane_section > 3
        tolllane_section = 1;
    end
    
    tmppos = get_posintrack(track, 1, 0, laneidx, 0);
    
    x = (4-carid)*5*10^3;
    y = tmppos(2);
    carpos = [x y 0];
    othercars.car{carid}.goallane = randi(3);
    
    if laneidx <= 5
        if othercars.car{carid}.goallane == 1
            othercars.car{carid}.crossflg = 0;
        else
            othercars.car{carid}.crossflg = 1;
        end
    elseif laneidx <= 10
        if othercars.car{carid}.goallane == 2
            othercars.car{carid}.crossflg = 0;
        else
            othercars.car{carid}.crossflg = 1;
        end
    else
        if othercars.car{carid}.goallane == 3
            othercars.car{carid}.crossflg = 0;
        else
            othercars.car{carid}.crossflg = 1;
        end
    end

    othercars.car{carid}.flgPlaza = 0; % 0:on straight lane ,1:on plaza lane
    othercars.car{carid}.flgIDM = 0; % 0:before controlling by IDM ,1:velocity control by IDM (both are on condition in the plaza)
    othercars.car{carid}.angry = 0;
    segidx = 1;
    othercars = add_othercars(othercars, carpos, [10000 0], 'normal',1, laneidx, segidx); % 10000 mm/s = 36 km/h
end


%---- setting for extra cars
%{
for carid = nr_cars+1:othercars.MAX_NRCAR
    tmp = mod(carid,2);
    if tmp == 0
       laneidx= 1;
    else
       laneidx= 3;
    end
    carpos = get_posintrack(track, 1, 0, laneidx, 0);
    othercars = add_othercars(othercars, carpos, [30000 0], 'normal',1, laneidx, 1); % 10000 mm/s = 36 km/h
end
othercars.n = nr_cars;
%}

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