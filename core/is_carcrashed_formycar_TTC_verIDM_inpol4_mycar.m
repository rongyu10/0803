function [idx_crashcar, t_crashcar, pos_crashcar, idx_frontCar] = is_carcrashed_formycar_TTC_verIDM_inpol4_mycar(othercars, time_TTC, step_TTC, mycar)

[idx_nearCar, idx_frontCar] = get_nearCar(mycar, othercars);
idx_crashcar = []; % 0:mycar 1~:number of othercar
t_crashcar = [];
pos_crashcar = [];

if isempty(idx_nearCar)
    return
else
    
    for t = 0:step_TTC:time_TTC
        if(~isempty(idx_nearCar))
            nr_cars = length(idx_nearCar);
            for i = 1:nr_cars
                if find(idx_crashcar == idx_nearCar(i))
                    continue
                end
                mycar_posEst = update_pos(mycar.pos, mycar.vel, t);
                mycar_bdEst = get_car4point(mycar_posEst, mycar.W + 1000, mycar.H + 1000);
                
                othercars_posEst = update_pos(othercars.car{idx_nearCar(i)}.pos, othercars.car{idx_nearCar(i)}.vel, t);
                othercars_bdEst = get_car4point(othercars_posEst, othercars.car{idx_nearCar(i)}.W, othercars.car{idx_nearCar(i)}.H);
                
                in = inpolygon(mycar_bdEst(:,1), mycar_bdEst(:,2), othercars_bdEst(:,1), othercars_bdEst(:,2));
                %----
                if any(in, 1)
                    idx_crashcar = idx_nearCar(i);
                    t_crashcar = t;
                    pos_crashcar = mycar_posEst;
                    break;
                end
                %----
            end
            if any(in, 1)
                break;
            end
        end
    end
end

end


function [idx_nearCar, idx_frontCar] = get_nearCar(mycar, othercars) % get the number of othercars in front of mycar and getting close to mycar

DISTANCE = 30*10^3;     % 30m

mycar_pos = mycar.pos(1:2);
nr_cars = othercars.n;

idx_nearCar =[];
idx_frontCar =[];
for i=1:nr_cars
    
    if mycar.pos(1) > othercars.car{i}.pos(1)
        continue
    end
    
    pos = othercars.car{i}.pos(1:2);
    diff= pos - mycar_pos;
    if norm(diff) < DISTANCE
      idx_frontCar= [idx_frontCar;i];
    end
    
%     if (mycar.pos(2) - othercars.car{i}.pos(2)) * (mycar.pos(3) - othercars.car{i}.pos(3)) > 0 % if both cars head to opposite direction
%         continue
%     end
    
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
