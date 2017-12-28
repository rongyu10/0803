function othercars = init_othercars()
% INITIALIZE OTHER CARS

%MAX_NRCAR = 10;
MAX_NRCAR = 100;  % mod by kumano
othercars.MAX_NRCAR = MAX_NRCAR;
othercars.n    = 0;
othercars.car  = cell(100, 1);
[W, H]    = get_carsize();
for i = 1:MAX_NRCAR
    othercars.car{i}.pos = [1E10 1E10 0]; 
    othercars.car{i}.vel = [0 0]; 
    othercars.car{i}.W   = W;
    othercars.car{i}.H   = H;
    othercars.car{i}.bd  = get_carshape(othercars.car{i}.pos, W, H);
    
    %--kumano----
    othercars.car{i}.save.pos = [1E10 1E10 0]; 
    othercars.car{i}.save.vel = [0 0];
    othercars.car{i}.save.track_idx = -1;
    othercars.car{i}.save.lane_idx  = -1;
    othercars.car{i}.save.seg_idx   = -1;
    %------------
    othercars.car{i}.paths  = [];
    othercars.car{i}.accele = [];
end
