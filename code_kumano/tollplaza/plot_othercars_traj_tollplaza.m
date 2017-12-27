function ems = plot_othercars_traj_tollplaza(traj_othercars)
% PLOT LANE CHANGE PATH
persistent fist_flag h
if isempty(fist_flag)
    fist_flag = true;
end

iclk = clock;
if fist_flag
   fist_flag = false;
   
   traj   = [];

   n_traj = traj_othercars.n;
   for i = 1:n_traj
       traj   = [traj;traj_othercars.route{i,1}; NaN NaN];
   end
   
   h.lane_ch = plot(traj(:, 1), traj(:, 2), ':' ...
               , 'Color', [0 1 1], 'LineWidth', 1);
end 

ems = etime(clock, iclk)*1000;

end