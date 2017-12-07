function [idx_crossingcar, arr_t_mycar, arr_t_othercar, dist_section_mycar] = detect_crossing_car(othercars, mycar, max_intersect_point)
% detect othercar crossing within 60m (currently exists within 30m from mycar)


idx_nearCar = get_nearCar(mycar, othercars);
% idx_nearCar = get_frontCar(mycar, othercars);
idx_crossingcar = [];
arr_t_mycar = [];
arr_t_othercar = [];
dist_section_mycar = [];
% rel_degree_crossingcar = [];

if isempty(idx_nearCar)
    return
else
    
    nr_cars = length(idx_nearCar);
    for i = 1:nr_cars
        
        if ~isempty(find(idx_crossingcar == idx_nearCar(i), 1)) % if the target index is already detected
            continue
        end
        
        [theta_mycar2other,~] = cart2pol(othercars.car{idx_nearCar(i)}.pos(1) - mycar.pos(1), othercars.car{idx_nearCar(i)}.pos(2) - mycar.pos(2));
        
        if (mycar.pos(3) - theta_mycar2other*180/pi)*(othercars.car{idx_nearCar(i)}.pos(3) - mycar.pos(3)) >= 0
            crosspoint = zeros(1,2);
            
            % calculate crossing point
            if mycar.pos(2) > othercars.car{idx_nearCar(i)}.pos(2) % if mycar exists over othercar
                for j = 1:201
                    if mycar.pathTranslated(j,2) < othercars.car{idx_nearCar(i)}.pos(2) + sin(othercars.car{idx_nearCar(i)}.pos(3)*180/pi) * 175 * (j-1) / 200
                        crosspoint = [mycar.pathTranslated(j,1), mycar.pathTranslated(j,2)];
                        break;
                    end
                end
            else  % if mycar exists under othercar
                for j = 1:201
                    if mycar.pathTranslated(j,2) > othercars.car{idx_nearCar(i)}.pos(2) + sin(othercars.car{idx_nearCar(i)}.pos(3)*180/pi) * 175 * (j-1) / 200
                        crosspoint = [mycar.pathTranslated(j,1), mycar.pathTranslated(j,2)];
                        break;
                    end
                end
            end
            
            % if crossing point exists within 60m, calculate each arrival time
            if ~isempty(crosspoint)
                dist_section_mycar = norm(crosspoint - mycar.pos(1:2));
                
                if  dist_section_mycar < max_intersect_point
                    if isempty(idx_crossingcar)
                        idx_crossingcar = idx_nearCar(i);
                        arr_t_mycar = dist_section_mycar / mycar.vel(1);
                        arr_t_othercar = norm(crosspoint - othercars.car{idx_nearCar(i)}.pos(1:2)) / othercars.car{idx_nearCar(i)}.vel(1);
                    elseif othercars.car{idx_nearCar(i)}.pos(1) < othercars.car{idx_crossingcar}.pos(1)
                        idx_crossingcar = idx_nearCar(i);
                        arr_t_mycar = dist_section_mycar / mycar.vel(1);
                        arr_t_othercar = norm(crosspoint - othercars.car{idx_nearCar(i)}.pos(1:2)) / othercars.car{idx_nearCar(i)}.vel(1);
                    end
                end
                
            end
            
        end
        
    end

end

end


function idx_nearCar = get_nearCar(mycar, othercars) % get the number of othercars in front of mycar and close to mycar

% DISTANCE = mycar.vel(1)*3;     % distance running in 3 seconds
DISTANCE = mycar.detect_rect_length;
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
