function othercars = respawn_othercars_tollplaza(othercars,road, sim)
% RESPAWN OTHERCARS outside the course 
% made by kumano

%----- Reset start position--
for i = 1:othercars.n
    if othercars.car{i}.pos(1) > road.xmax
        for j=1:nnz(othercars.car_nr(othercars.car{i}.goallane,:))
            if j ~= nnz(othercars.car_nr(othercars.car{i}.goallane,:))
                othercars.car_nr(othercars.car{i}.goallane, j) = othercars.car_nr(othercars.car{i}.goallane, j+1);
            else
                othercars.car_nr(othercars.car{i}.goallane, j) = 0;
            end
        end
        othercars.car{i}.flgPlaza = 0;
        othercars.car{i}.pos(1) = -10000;
        othercars.car{i}.vel = [0 0];
    end
end
%----------------------------
end