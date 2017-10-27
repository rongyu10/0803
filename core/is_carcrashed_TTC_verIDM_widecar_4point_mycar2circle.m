function [idx_nearCar, idx_crashcar, t_crashcar, pos_crashcar, est_mycar] = is_carcrashed_TTC_verIDM_widecar_4point_mycar2circle(othercars, idx, time_TTC, step_TTC, mycar)

idx_nearCar = get_nearCar(othercars,idx); % (multiple number of cars in this)
est_mycar = get_nearMyCar(mycar, othercars, idx);
idx_crashcar = []; % 0:mycar 1~:number of othercar (only 1 car in this)
t_crashcar = [];
pos_crashcar = [];

if isempty(idx_nearCar) && isempty(est_mycar)
    return
else
    
    for t = 0:step_TTC:time_TTC
        
        % if there are any othercars near index car
        if(~isempty(idx_nearCar))
            nr_cars = length(idx_nearCar);
            for i = 1:nr_cars
                if find(idx_crashcar == idx_nearCar(i))
                    continue
                end
                mycar_posEst = update_pos(othercars.car{idx}.pos, othercars.car{idx}.vel, t);
                mycar_bdEst = get_car4point(mycar_posEst, othercars.car{idx}.W, othercars.car{idx}.H);
                
                othercars_posEst = update_pos(othercars.car{idx_nearCar(i)}.pos, othercars.car{idx_nearCar(i)}.vel, t);
                othercars_bdEst = get_car2point(othercars_posEst, othercars.car{idx_nearCar(i)}.W, othercars.car{idx_nearCar(i)}.H);
                
                flgCollide = 0;
                for idx_point = 1:4
                    if norm(mycar_bdEst(idx_point,:) - othercars_bdEst(1,:)) < 2000
                        %fprintf(1, 'Front point[%d] ', idx_point);
                        flgCollide = 1;
                        break;
                    elseif norm(mycar_bdEst(idx_point,:) - othercars_bdEst(2,:)) < 2000
                        %fprintf(1, 'Rear point[%d] ', idx_point);
                        flgCollide = 1;
                        break;
                    end
                end
                %in = inpolygon(mycar_bdEst(:,1), mycar_bdEst(:,2), othercars_bdEst(:,1), othercars_bdEst(:,2));
                %----
                if flgCollide == 1
                    idx_crashcar = idx_nearCar(i);
                    t_crashcar = t;
                    pos_crashcar = mycar_posEst;
                    break;
                end
                %----
            end
            if flgCollide == 1
                break;
            end
        end
        
        % if there is mycar near index car
        if(~isempty(est_mycar))
            
            mycar_posEst = update_pos(othercars.car{idx}.pos, othercars.car{idx}.vel, t);
            mycar_bdEst = get_carshape(mycar_posEst, othercars.car{idx}.W + 1000, othercars.car{idx}.H + 1000);
            
            othercars_posEst = update_pos(mycar.pos, mycar.vel, t);
            othercars_bdEst = get_carshape(othercars_posEst, mycar.W + 1000, mycar.H + 1000);
            in = inpolygon(mycar_bdEst(:,1), mycar_bdEst(:,2), othercars_bdEst(:,1), othercars_bdEst(:,2));
            %----
            if any(in, 1) && othercars.car{idx}.pos(1) < mycar.pos(1)
                idx_crashcar = 0;
                t_crashcar = t;
                pos_crashcar = mycar_posEst;
                break;
            end
            %----
        end
        
    end
end

end

function est_mycar = get_nearMyCar(mycar, othercars, idx)

est_mycar = [];
DISTANCE = othercars.car{idx}.vel(1)*3;

if othercars.car{idx}.pos(1) > mycar.pos(1)
    return
end

pos = othercars.car{idx}.pos(1:2);
diff= pos - mycar.pos(1:2);

if norm(diff) < DISTANCE
    est_mycar = true;
end

end

function idx_nearCar = get_nearCar(othercars,idx) % get the number of othercars in front of mycar and getting close to object car

DISTANCE = othercars.car{idx}.vel(1)*3;

mycar_pos = othercars.car{idx}.pos(1:2);
nr_cars = othercars.n;

idx_nearCar =[];
for i=1:nr_cars
    if i == idx % if calculation target is myself
        continue
    end
    
    if othercars.car{idx}.pos(1) > othercars.car{i}.pos(1)
        continue
    end
%     
%     if (othercars.car{idx}.pos(2) - othercars.car{i}.pos(2)) * (othercars.car{idx}.pos(3) - othercars.car{i}.pos(3)) > 0 && (othercars.car{idx}.goallane ~= othercars.car{i}.goallane) % if both cars head to opposite direction(eliminate same goallane)
%         continue
%     end
% 
%     pos = othercars.car{i}.pos(1:2);
%     diff= pos - mycar_pos;
%     if norm(diff) < DISTANCE
%       idx_nearCar= [idx_nearCar;i];
%     end
    
    if abs(othercars.car{idx}.pos(1)-othercars.car{i}.pos(1)) < DISTANCE
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
