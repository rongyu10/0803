function pedestrians = init_pedestrians()
% INITIALIZE pedestrians

%MAX_PEOPLE = 10;
MAX_PEOPLE = 20;  % mod by kumano
pedestrians.MAX_PEOPLE = MAX_PEOPLE;
pedestrians.n    = 0;
pedestrians.person  = cell(100, 1);
D = get_humansize();

for i = 1:MAX_PEOPLE
    pedestrians.person{i}.pos = [1E10 1E10 0]; 
    pedestrians.person{i}.vel = [0 0];
    pedestrians.person{i}.vel_save = [0 0];
    pedestrians.person{i}.pos_save = [1E10 1E10 0];    
    pedestrians.person{i}.D   = D;
    pedestrians.person{i}.bd  = get_personshape(pedestrians.person{i}.pos, D);
    pedestrians.person{i}.paths = [];
    pedestrians.person{i}.stoptime    = [];
    pedestrians.person{i}.noobstacletime= [];
    
    % RANGEFINDER SENSOR
    pedestrians.person{i}.r         = 0;
    pedestrians.person{i}.rfs_dist  = 5000; %
    pedestrians.person{i}.rfs_deg   = 60;   % 270
    pedestrians.person{i}.nr_rfs    = 6;   % 19
    pedestrians.person{i}.rfs_degs  = ...
     linspace(-pedestrians.person{i}.rfs_deg/2, pedestrians.person{i}.rfs_deg/2, pedestrians.person{i}.nr_rfs);
    pedestrians.person{i}.rfs_dists = pedestrians.person{i}.rfs_dist*ones(1, pedestrians.person{i}.nr_rfs);
end
