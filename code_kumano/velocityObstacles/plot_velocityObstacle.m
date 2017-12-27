function ems = plot_velocityObstacle(mycar, pedestrians, varargin)
%
%  varargin{1}: maximum radius for searching obstacle 
%  varargin{2}: maximum speed for velocity obstacle area 


persistent fist_flag h
if isempty(fist_flag)
    fist_flag = true;
end

iclk = clock;

if (nargin == 3)
   DIST_PLOT = varargin{1};
else
   DIST_PLOT = 15*10^3;
end


pos = mycar.pos(1:2);

% PLOT
if fist_flag
    fist_flag = false;    
    for i = 1:pedestrians.n
        [~, vo_global] = get_velocityObstacle(mycar, pedestrians.person{i});

        h.veloObst{i} = fill(vo_global(1,:),vo_global(2,:),'b');
        h.veloObst{i}.FaceAlpha = 0.2;
        pos_pedestrian = pedestrians.person{i}.pos(1:2);
        if norm(pos_pedestrian-pos,2) > DIST_PLOT
           h.veloObst{i}.Visible = 'off';
        end
    end
            
else
    % update Velocity Obstacle plot
    for i = 1:pedestrians.n
        [~, vo_global] = get_velocityObstacle(mycar, pedestrians.person{i});
        h.veloObst{i}.Vertices = vo_global';
        pos_pedestrian = pedestrians.person{i}.pos(1:2);
        if norm(pos_pedestrian-pos,2) > DIST_PLOT
           h.veloObst{i}.Visible = 'off';
        else
           h.veloObst{i}.Visible = 'on';
        end

    end
    
end
ems = etime(clock, iclk)*1000;

end
