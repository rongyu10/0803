function ems = plot_goalgate_tollplaza(track, idx_goalgate, varargin)
persistent first_flag save_idx_gate h htxt
if isempty(first_flag)
    first_flag = true;
end

tollGateFlag  = true;
rotateAngle   = 0;
%--- for OFF-RAMP ---------
if (nargin == 4)&&(strcmp(varargin{1},'offramp'))
   tollGateFlag  = false;
   rotateAngle   = varargin{2};
elseif (nargin == 3)&&(strcmp(varargin{1},'offramp'))
   tollGateFlag  = false;
   rotateAngle   = 0;    
end
%-------------------------

iclk = clock;

normalcolor = 0.3*[1 1 1];
goalcolor = 'c';

if first_flag
    first_flag    = false;
    save_idx_gate = idx_goalgate;
    nr_lane = track.nr_lane;
    width   = track.width;

    ps         = track.seg{1}.p;
    interval_Y = width/nr_lane;
    dY  = [0 interval_Y];
    
    for i = 1:nr_lane
        if tollGateFlag % toll gate
            p1 = ps(4, :) - dY*(i-1);
            p2 = ps(4, :) - dY*i;
            p3 = ps(3, :) - dY*i;
            p4 = ps(3, :) - dY*(i-1);
        else % off ramp
            p1 = ps(1, :);
            p2 = ps(2, :);
            p3 = ps(3, :);
            p4 = ps(4, :);
        end
        bd = [p1;p2;p3;p4;p1];
        centPt = (p1 + p3)*0.5;
        %--- PLOT GOAL GATE -------
        if i ==  idx_goalgate
            h.gate{i} = fill(bd(:, 1), bd(:, 2), goalcolor,'EdgeColor', 'k', 'LineWidth', 1);
            htxt.gate{i} = text(centPt(1),centPt(2),'GOAL','Color','b','HorizontalAlignment','center','Rotation',rotateAngle);
        else
            h.gate{i} = fill(bd(:, 1), bd(:, 2), normalcolor,'EdgeColor', 'k', 'LineWidth', 1);
            htxt.gate{i} = text(centPt(1),centPt(2),'GOAL','Color','b','HorizontalAlignment','center','Rotation',rotateAngle);
            htxt.gate{i}.Visible = 'off';
        end
        %--------------------------
    end    
else
    % updating goal location 
    if idx_goalgate ~= save_idx_gate
       h.gate{save_idx_gate}.FaceColor = normalcolor;
       htxt.gate{save_idx_gate}.Visible = 'off';
       h.gate{idx_goalgate}.FaceColor  = goalcolor;
       htxt.gate{idx_goalgate}.Visible = 'on';
       save_idx_gate = idx_goalgate;
    end

end

ems = etime(clock, iclk)*1000;

end