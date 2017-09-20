function [othercars, table_same_lane] = respawn_othercars_tollplaza(othercars,road, table_same_lane)
% RESPAWN OTHERCARS outside the course 
% made by kumano

%----- Reset start position--
for i = 1:othercars.n
    if othercars.car{i}.pos(1) > road.xmax && othercars.car{i}.pos(1) < 330
        for j=1:nnz(table_same_lane(othercars.car{i}.goallane,:))
            if j ~= nnz(table_same_lane(othercars.car{i}.goallane,:))
                table_same_lane(othercars.car{i}.goallane, j) = table_same_lane(othercars.car{i}.goallane, j+1);
            else
                table_same_lane(othercars.car{i}.goallane, j) = 0;
            end
        end
        othercars.car{i}.flgPlaza = 0;
        othercars.car{i}.pos(1) = 340*10^3;
        othercars.car{i}.vel = [0 0];
    end
end
%----------------------------
end