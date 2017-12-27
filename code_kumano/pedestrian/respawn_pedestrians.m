function pedestrians = respawn_pedestrians(pedestrians,track)
% RESPAWN OTHERCARS outside the course 
% made by kumano

for i = 1:pedestrians.n
    
    if pedestrians.person{i}.pos(1) >= track.xmax
        pedestrians.person{i}.pos = reflection(pedestrians.person{i}.pos, '-x');
    elseif pedestrians.person{i}.pos(1) <= track.xmin
        pedestrians.person{i}.pos = reflection(pedestrians.person{i}.pos, '+x');
    elseif pedestrians.person{i}.pos(2) >= track.ymax
        pedestrians.person{i}.pos = reflection(pedestrians.person{i}.pos, '-y');
    elseif pedestrians.person{i}.pos(2) <= track.ymin
        pedestrians.person{i}.pos = reflection(pedestrians.person{i}.pos, '+y');
    end
    
end

end

function newpos = reflection(pos, norm)

newpos = pos;

%---- direction of velocity------
dx = cos(pos(3)*pi/180);
dy = sin(pos(3)*pi/180);
direction = [dx, dy];
%--------------------------------

if norm=='+x'
    wall = [1, 0];
    innerpord = direction*wall';
    if innerpord < 0
       newpos(3) = 180-pos(3);
    else 
       newpos(3) = pos(3);
    end
elseif norm=='-x'
    wall = [-1, 0];
    innerpord = direction*wall';
    if innerpord < 0
       newpos(3) = 180-pos(3);
    else 
       newpos(3) = pos(3);
    end
elseif norm=='+y'
    wall = [0, 1];
    innerpord = direction*wall';
    if innerpord < 0
        newpos(3) = -pos(3);
    else
       newpos(3) = pos(3);
    end
    
elseif norm=='-y'
    wall = [0, -1];
    innerpord = direction*wall';
    if innerpord < 0
        newpos(3) = -pos(3);
    else
       newpos(3) = pos(3);
    end
end    

end