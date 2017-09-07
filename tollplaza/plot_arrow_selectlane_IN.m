function [h1, h2, h3] = plot_arrow_selectlane_IN(selectlane)

persistent first_flag h
if isempty(first_flag)
    first_flag = true;
end

if first_flag
    first_flag = false;
    [h1, h2, h3] = plot_arrow([315*10^3 77.5*10^3 - 5000*selectlane], 5000, 0, 'w', 1.5);
    h.targetarrow = [h1 h2 h3];
else
    [x1, y1, x2, y2, x3, y3] = get_arrow([315*10^3 77.5*10^3 - 5000*selectlane], 5000, 180, 'w', 1.5);
    h.targetarrow(1).XData = x1; h.targetarrow(1).YData = y1;
    h.targetarrow(2).XData = x2; h.targetarrow(2).YData = y2;
    h.targetarrow(3).XData = x3; h.targetarrow(3).YData = y3;

end


