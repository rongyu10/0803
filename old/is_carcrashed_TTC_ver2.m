function [idx_crashcar, t] = is_carcrashed_TTC_ver2(othercars, idx, time_TTC, step_TTC)

idx_nearCar = get_nearCar(othercars,idx);
idx_crashcar = [];
t = [];
if isempty(idx_nearCar)
    return
else
    nr_cars = length(idx_nearCar);
    for t = step_TTC:step_TTC:time_TTC
        for i = 1:nr_cars
            mycar_posEst = update_pos(othercars.car{idx}.pos, othercars.car{idx}.vel, t);
            mycar_bdEst = get_carshape(mycar_posEst, othercars.car{idx}.W, othercars.car{idx}.H);
            mycarBD    = get_carLineData(mycar_bdEst);
            
            othercars_posEst = update_pos(othercars.car{idx_nearCar(i)}.pos, othercars.car{idx_nearCar(i)}.vel, t);
            othercars_bdEst = get_carshape(othercars_posEst, othercars.car{idx_nearCar(i)}.W, othercars.car{idx_nearCar(i)}.H);
            othercarBD = get_othercarBD(othercars_bdEst);
            P = InterX(mycarBD, othercarBD);
            %----
            if ~isempty(P)
                idx_crashcar = [idx_crashcar;idx_nearCar(i)];
            end
            %----
            
        end
        if ~isempty(idx_crashcar)
            break;
        end
    end
end

end


function idx_nearCar = get_nearCar(othercars,idx)

DISTANCE = 20*10^3;     % 30m

mycar_pos = othercars.car{idx}.pos(1:2);
nr_cars = othercars.n;

idx_nearCar =[];
for i=1:nr_cars
    if i == idx
        continue
    end
    pos = othercars.car{i}.pos(1:2);
    diff= pos - mycar_pos;
    if norm(diff) < DISTANCE
      idx_nearCar= [idx_nearCar;i];
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
