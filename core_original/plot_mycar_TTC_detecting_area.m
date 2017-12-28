function ems = plot_mycar_TTC_detecting_area(est_squareX, est_squareY)
% PLOT MYCAR
persistent first_flag h
if isempty(first_flag)
    first_flag = true;
end
iclk = clock;

if first_flag
    first_flag = false;
%     c = cos(mycar.pos(3)*pi/180);
%     s = sin(mycar.pos(3)*pi/180);
%     squareX = [mycar.pos(1)+2500*s+6000*c mycar.pos(1)+2500*s+30000*c mycar.pos(1)-2500*s+30000*c mycar.pos(1)-2500*s+6000*c mycar.pos(1)+2500*s+6000*c];
%     squareY = [mycar.pos(2)-2500*c+6000*s mycar.pos(2)-2500*c+30000*s mycar.pos(2)+2500*c+30000*s mycar.pos(2)+2500*c+6000*s mycar.pos(2)-2500*c+6000*s];
    h.carbd = plot(est_squareX, est_squareY, 'g', 'LineWidth', 1);   
else
%     c = cos(mycar.pos(3)*pi/180);
%     s = sin(mycar.pos(3)*pi/180);
%     h.carbd.XData = [mycar.pos(1)+2500*s+6000*c mycar.pos(1)+2500*s+30000*c mycar.pos(1)-2500*s+30000*c mycar.pos(1)-2500*s+6000*c mycar.pos(1)+2500*s+6000*c];
%     h.carbd.YData = [mycar.pos(2)-2500*c+6000*s mycar.pos(2)-2500*c+30000*s mycar.pos(2)+2500*c+30000*s mycar.pos(2)+2500*c+6000*s mycar.pos(2)-2500*c+6000*s];
    h.carbd.XData = est_squareX;
    h.carbd.YData = est_squareY;
end
ems = etime(clock, iclk)*1000;