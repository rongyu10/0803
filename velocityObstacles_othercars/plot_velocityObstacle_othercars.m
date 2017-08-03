function ems = plot_velocityObstacle_othercars(mycar, othercars)
persistent fist_flag h
if isempty(fist_flag)
    fist_flag = true;
end

iclk = clock;
DIST_PLOT = 25*10^3;

pos = mycar.pos(1:2);

% PLOT
if fist_flag
    fist_flag = false;    
    for i = 1:othercars.n
        [~, vo_global] = get_velocityObstacle_othercars(mycar, othercars.car{i});

        h.veloObst{i} = fill(vo_global(1,:),vo_global(2,:),'b');
        h.veloObst{i}.FaceAlpha = 0.2;
        pos_othercar = othercars.car{i}.pos(1:2);
        if norm(pos_othercar-pos,2) > DIST_PLOT
           h.veloObst{i}.Visible = 'off';
        end
    end
            
else
    % update Velocity Obstacle plot
    for i = 1:othercars.n
        [~, vo_global] = get_velocityObstacle_othercars(mycar, othercars.car{i});
        h.veloObst{i}.Vertices = vo_global';
        pos_othercar = othercars.car{i}.pos(1:2);
        if norm(pos_othercar-pos,2) > DIST_PLOT
           h.veloObst{i}.Visible = 'off';
        else
           h.veloObst{i}.Visible = 'on';
        end

    end
    
end
ems = etime(clock, iclk)*1000;

end
