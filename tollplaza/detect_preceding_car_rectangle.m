function [idx_precedingcar, rel_degree_precedingcar] = detect_preceding_car_rectangle(othercars, mycar, sim)
% detect preceding car of IDM by using forward rectangle (relative angle is smaller than 5 degree)


idx_nearCar = get_nearCar(mycar, othercars);
% idx_nearCar = get_frontCar(mycar, othercars);
idx_precedingcar = [];
rel_degree_precedingcar = [];

persistent iPrecede;
if isempty(iPrecede)
    iPrecede = 0;
end

if isempty(idx_nearCar)
    return
else
    clk_TTC = clock;
    
    
    nr_cars = length(idx_nearCar);
    for i = 1:nr_cars
        
        rel_degree = mycar.pos(3) - othercars.car{idx_nearCar(i)}.pos(3);
%         if abs(rel_degree) > 5 % if the relative degree is over 5 degree
%             continue
%         end
        
        % judge whether object othercar is in mycar's detecting area---------------
        in = inpolygon(othercars.car{idx_nearCar(i)}.pos(1), othercars.car{idx_nearCar(i)}.pos(2), mycar.squareX, mycar.squareY);
        
        if any(in, 1)
            if isempty(idx_precedingcar)
                idx_precedingcar = idx_nearCar(i);
                rel_degree_precedingcar = rel_degree;
            elseif othercars.car{idx_nearCar(i)}.pos(1) < othercars.car{idx_precedingcar}.pos(1)
                idx_precedingcar = idx_nearCar(i);
                rel_degree_precedingcar = rel_degree;
            end
        end
        %------------------------------
        
    end
    
    if ~isempty(idx_precedingcar)
        iPrecede = iPrecede + sim.T;
        if iPrecede < mycar.time_detect_precedingcar
            idx_precedingcar = [];
        end
    else
        iPrecede = 0;
    end
    
    
    ms_TTC  = etime(clock, clk_TTC)*1000;
    %fprintf(2, 'TTC calculating = [%d]msec\n', ms_TTC);
end

end


function idx_nearCar = get_nearCar(mycar, othercars) % get the number of othercars in front of mycar and close to mycar

% DISTANCE = mycar.vel(1)*3;     % distance running in 3 seconds
DISTANCE = mycar.detect_length;
mycar_pos = mycar.pos(1:2);
nr_cars = othercars.n;

idx_nearCar =[];
for i=1:nr_cars
    
    if abs(mycar.pos(1)-othercars.car{i}.pos(1)) < DISTANCE
        pos = othercars.car{i}.pos(1:2);
        diff= pos - mycar_pos;
        
        if norm(diff) < DISTANCE
            idx_nearCar= [idx_nearCar;i];
        end
    end
    
end

end

function idx_nearPerson = get_nearPerson(mycar,pedestrians)

DISTANCE = 10*10^3;     % 20m

mycar_pos      = mycar.pos(1:2);
nr_pedestrians = pedestrians.n;

idx_nearPerson =[];
for i=1:nr_pedestrians
    pos = pedestrians.person{i}.pos(1:2);
    diff= pos - mycar_pos;
    if norm(diff) < DISTANCE
        idx_nearPerson= [idx_nearPerson;i];
    end
end

end

function othercarBD = get_othercarBD(othercars_bdEst)

othercarBD = zeros(2, 1E3);
nr_bd      = 0;
nr_curr_bd = 7;

lineData = get_carLineData(othercars_bdEst);
tmpBD = [lineData [NaN ; NaN]];
othercarBD(:,nr_bd+1: nr_bd+nr_curr_bd) = tmpBD;
nr_bd = nr_bd + nr_curr_bd;

othercarBD = othercarBD(:,1:nr_bd);

end

function pedestrianBD = get_pedestrianBD(pedestrians,idx_nearPerson)

nr_pedestrians = length(idx_nearPerson);
pedestrianBD = [];
for i = 1:nr_pedestrians
    lineData = pedestrians.person{idx_nearPerson(i)}.bd.circle;
    tmpBD = [lineData' [NaN ; NaN]];
    pedestrianBD = [pedestrianBD,tmpBD];
end

end




function lineData = get_carLineData(car)

lineData = [car(1,1),car(2,1),car(4,1),car(5,1),car(7,1),car(8,1);...
    car(1,2),car(2,2),car(4,2),car(5,2),car(7,2),car(8,2)];

end
