function othercars = respawn_othercars(othercars,track)
% RESPAWN OTHERCARS outside the course 
% made by kumano

for i = 1:othercars.n
    if othercars.car{i}.pos(1) > track.xmax
        othercars.car{i}.pos = get_posintrack(track, 1, 0, 2, 0); % respawn from 2nd lane (no offset)
    end
end
