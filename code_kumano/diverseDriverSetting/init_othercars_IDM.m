function othercars = init_othercars_IDM(othercars,desired_velocity, ratio_polite_driver)
% INITIALIZE Intelligent Driver Model for OTHER CARS

%---- IDM default setting---------------------------
v0    = desired_velocity;   % 5000 desired_velocity[mm/s]
T     = 1.5;       % 1.6  safetime_headway[s]
a_max = 1500;      % 730  maximum acceleration [mm/s^2]
b_max = 3000;      % 1670 confotable decceletion  [mm/s^2]
delta = 4.0;       % 4.0 acceleration exponent [-]
s0    = 2000;      % 2000 linear jam distance [mm]
%---------------------------------------------------

for i = 1:othercars.MAX_NRCAR
    othercars.car{i}.IDM.v0     = v0;
    othercars.car{i}.IDM.T      = T; % +(rand()-0.5)*1.2;

    othercars.car{i}.IDM.a_max  = a_max;
    othercars.car{i}.IDM.b_max  = b_max;
    othercars.car{i}.IDM.delta  = delta;
    othercars.car{i}.IDM.s0     = s0;
    othercars.car{i}.IDM.angry  = 0;
    
    othercars.car{i}.IDM.polite = 0;

%    rand_polite = rand();
%    if rand_polite < ratio_polite_driver
%        othercars.car{i}.IDM.polite = 1;
%    else
%        othercars.car{i}.IDM.polite = 0;
%    end
%     if (i ==1)|| (i ==2)
%         othercars.car{i}.IDM.polite = 1;
%     else
%         othercars.car{i}.IDM.polite = 0;
%     end

    othercars.car{i}.IDM.ratio_decel = 0.5 + rand()*0.5;
    
    %if mod(i,2)==0
    %   othercars.car{i}.IDM.polite = 1;
    %else
       %othercars.car{i}.IDM.polite = 0;
       %othercars.car{i}.IDM.T      = T -0.6;
       %othercars.car{i}.IDM.a_max  = a_max + 1000;
       %othercars.car{i}.IDM.s0     = s0 - 1000;
end

%--- for uniform Polite sampling -----
n_cars    = 10;
n_polite  = round(n_cars*ratio_polite_driver);
%idx_polite= randsample(n_cars,n_polite,false); % NEED Statistics_Toolbox!!
idx_polite=my_randsample(n_cars, n_polite);
for i = 1:n_polite
   idx = idx_polite(i);
   othercars.car{idx}.IDM.polite = 1;
end
%-------------------------------------

end

