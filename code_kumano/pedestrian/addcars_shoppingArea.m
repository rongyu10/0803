function othercars = addcars_shoppingArea(othercars, track, nr_cars)
% ADD CARS IN RANDOM POSTIONS
VERBOSE = 0;

%{
for carid = 1:nr_cars-1
    if carid <= track.nr_seg
       laneidx= 2;
    end
    if carid > track.nr_seg
       laneidx= 3;
    end
    segidx = mod(carid-1,track.nr_seg)+1;
    seglen = track.seg{segidx}.d;
    randoffset = seglen*rand*0.5;
    %randoffset = seglen*0.5;
    carpos = get_posintrack(track, segidx, randoffset, laneidx, 0);
%    othercars = add_othercars(othercars, carpos, [15000 0], 'normal');
    othercars = add_othercars(othercars, carpos, [10000 0], 'normal'); % 10000 mm/s = 36 km/h
    %othercars = add_othercars(othercars, carpos, [0 0], 'stop');
end
%}

%　駐車車両の処理
%---1台目
laneidx= 1;
segidx = 4;
seglen = track.seg{segidx}.d;
randoffset = seglen*rand*0.5;
%randoffset = seglen*0.5;
carpos = get_posintrack(track, segidx, randoffset, laneidx, 0);
othercars = add_othercars(othercars, carpos, [0 0], 'stop');
othercars.car{1, 1}.pos(2) = othercars.car{1, 1}.pos(2) +2000; % y-coord translation

%---2台目
laneidx= 1;
segidx = 8;
seglen = track.seg{segidx}.d;
randoffset = seglen*rand*0.5;
%randoffset = seglen*0.5;
carpos = get_posintrack(track, segidx, randoffset, laneidx, 180);
othercars = add_othercars(othercars, carpos, [0 0], 'stop');
othercars.car{2, 1}.pos(2) = othercars.car{2, 1}.pos(2) -2000; % y-coord translation

end



function othercars = add_othercars(othercars, pos, vel, ctrlmode)
if nargin == 3
    ctrlmode = 'normal';
end
othercars.n = othercars.n + 1;
othercars.car{othercars.n}.pos = pos;
othercars.car{othercars.n}.vel = vel;
othercars.car{othercars.n}.ctrlmode = ctrlmode;
othercars.car{othercars.n}.paths = [];
end
