function ems = plot_mycar_detecting_area(mycar)
% PLOT MYCAR
persistent first_flag h
if isempty(first_flag)
    first_flag = true;
end
iclk = clock;

if first_flag
    first_flag = false;
    h.carbd = plot(mycar.squareX, mycar.squareY, 'r', 'LineWidth', 1);   
else
    h.carbd.XData = mycar.squareX;
    h.carbd.YData = mycar.squareY;
end
ems = etime(clock, iclk)*1000;