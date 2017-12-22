function [idx_precedingcar, rel_degree_precedingcar] = detect_preceding_car_rectangle(othercars, mycar)
% detect preceding car of IDM by using forward rectangle (relative angle is smaller than 5 degree)


idx_nearCar = get_nearCar(mycar, othercars);
% idx_nearCar = get_frontCar(mycar, othercars);
idx_precedingcar = [];
rel_degree_precedingcar = [];

if isempty(idx_nearCar)
    return
else
    clk_TTC = clock;
    
    
    nr_cars = length(idx_nearCar);
    for i = 1:nr_cars
        if ~isempty(find(idx_precedingcar == idx_nearCar(i), 1)) % if the target index is already detected
            continue
        end
        
        rel_degree = mycar.pos(3) - othercars.car{idx_nearCar(i)}.pos(3);
        
        if abs(rel_degree) > 5 % if the target index is already detected
            continue
        end
        
        % detect the index(and so on) of othercars in mycar's detecting area---------------
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
        
        %             if FLAG_OTHERCAR_INTENTION_EST
        %                 % detect the index(and so on) of othercars which detect mycar in its detecting area---------------
        %                 for j = 0:1
        %
        %                     Pos_est = update_pos(othercars_posEst, othercars.car{idx_nearCar(i)}.vel, j*3.0);
        %
        %                     left_right_point = get_car_futurepoint(Pos_est, mycar.W, 5000);
        %                     othercar_est_squareX(j+1) = left_right_point(1,1);
        %                     othercar_est_squareY(j+1) = left_right_point(1,2);
        %                     othercar_est_squareX(4-j) = left_right_point(2,1);
        %                     othercar_est_squareY(4-j) = left_right_point(2,2);
        %
        %                 end
        %                 othercar_est_squareX(5) = othercar_est_squareX(1);
        %                 othercar_est_squareY(5) = othercar_est_squareY(1);
        %                 in = inpolygon(mycar_posEst(1), mycar_posEst(2), othercar_est_squareX, othercar_est_squareY);
        %
        %                 if any(in, 1)
        %                     idx_observing_mycar = [idx_observing_mycar; idx_nearCar(i)];
        %                     angle_observing_mycar = [angle_observing_mycar; abs(mycar_posEst(3) - othercars_posEst(3))];
        %                     mycarpos_observing_mycar = [mycarpos_observing_mycar; mycar_posEst];
        %                     t_observing_mycar = [t_observing_mycar; t];
        %                 end
        %                 %------------------------------
        %             end
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
