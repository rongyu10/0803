function ems = plot_mycar_path_in_plaza(laneChangePath, mycar)
% PLOT MYCAR
persistent first_flag h
if isempty(first_flag)
    first_flag = true;
end
iclk = clock;

if first_flag
    first_flag = false;
    h.path = plot(laneChangePath{mycar.selectlane, mycar.save.lane_idx}(:,1), laneChangePath{mycar.selectlane, mycar.save.lane_idx}(:,2), ':c', 'LineWidth', 10);   
else
    h.path.XData = laneChangePath{mycar.selectlane, mycar.save.lane_idx}(:,1);
    h.path.YData = laneChangePath{mycar.selectlane, mycar.save.lane_idx}(:,2);
end
ems = etime(clock, iclk)*1000;