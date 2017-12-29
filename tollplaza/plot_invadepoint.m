function ems = plot_invadepoint(invadepoint)
% PLOT INVADE POINT
persistent first_flag h
if isempty(first_flag)
    first_flag = true;
end
iclk = clock;

if first_flag
    first_flag = false;
    h.invadepoint = plot(invadepoint(1), invadepoint(2), '-o');   
else
    h.invadepoint.XData = invadepoint(1);
    h.invadepoint.YData = invadepoint(2);
end
ems = etime(clock, iclk)*1000;