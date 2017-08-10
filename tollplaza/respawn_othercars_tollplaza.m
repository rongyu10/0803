function othercars = respawn_othercars_tollplaza(othercars,road, sim)
% RESPAWN OTHERCARS outside the course 
% made by kumano


%{
intvl = mod(sim.tick,10); % every 10 step
if intvl ==0
   if othercars.n < othercars.MAX_NRCAR
      othercars.n = othercars.n + 1;
   end
end
%}

%----- Reset start position--
for i = 1:othercars.n
    if othercars.car{i}.pos(1) > road.xmax
        laneidx = othercars.car{i}.save.lane_idx;  
        othercars.car{i}.pos = get_posintrack(road.track{1}, 1, 0, laneidx, 0); % respawn from 2nd lane (no offset)
        %othercars.car{i}.vel = othercars.car{i}.save.vel;  
        othercars.car{i}.vel = [10000 0];
    end
end
%----------------------------
end