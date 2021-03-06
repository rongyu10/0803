function mycar = init_mycar(pos,vel)
% INITIALIZE MY CAR
mycar.pos = pos;
%mycar.vel = [5000 0];
mycar.vel = vel;

[W, H] = get_carsize();
mycar.W = W; mycar.H = H;
mycar.bd = get_carshape(mycar.pos, mycar.W, mycar.H);

% RANGEFINDER SENSOR
mycar.r         = 0;
mycar.rfs_dist  = 15000;  % 20000
mycar.rfs_deg   = 360;     % 360
mycar.nr_rfs    = 25;      % 25
mycar.rfs_degs  = linspace(-mycar.rfs_deg/2, mycar.rfs_deg/2, mycar.nr_rfs);
mycar.rfs_dists = mycar.rfs_dist*ones(1, mycar.nr_rfs);



