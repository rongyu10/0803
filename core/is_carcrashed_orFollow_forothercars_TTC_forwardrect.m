function [idx_observedcar, t_observedcar, pos_mycarEst, pos_observedcarEst] = is_carcrashed_orFollow_forothercars_TTC_forwardrect(othercars, idx, time_TTC, step_TTC, mycar, laneChangePath)

idx_nearCar = get_nearCar(othercars,idx); % (multiple number of cars in this)
est_mycar = get_nearMyCar(mycar, othercars, idx);
idx_observedcar = []; % 0:mycar 1~:number of othercar (only 1 car in this)
t_observedcar = [];
pos_mycarEst = [];
pos_observedcarEst = [];

if isempty(idx_nearCar) && isempty(est_mycar)
    return
else
    
    for t = 0:step_TTC:time_TTC
        
        mycar_posEst(1) = othercars.car{idx}.pos(1) + othercars.car{idx}.vel(1)*t;
        
        if mycar_posEst(1) <= 100*10^3
            mycar_posEst(2) = othercars.car{idx}.pos(2);
        elseif mycar_posEst(1) <= 275*10^3
            %nData = size(laneChangePath{othercars.car{idx}.goallane, mycar.save.lane_idx},1);
            for idx_me = 1:201
                if mycar_posEst(1) - laneChangePath{othercars.car{idx}.goallane, othercars.car{idx}.save.lane_idx}(idx_me,1) < 0
                    mycar_posEst(1) = laneChangePath{othercars.car{idx}.goallane, othercars.car{idx}.save.lane_idx}(idx_me,1);
                    mycar_posEst(2) = laneChangePath{othercars.car{idx}.goallane, othercars.car{idx}.save.lane_idx}(idx_me,2);
                    break;
                end
            end
        else
            mycar_posEst(2) = (77.5-othercars.car{idx}.goallane*5.0)*10^3;
        end
        
        
        % make square of detecting area for IDM(TTCver)-----------------------------------
        est_squareX = zeros(1,13);
        est_squareY = zeros(1,13);
        
        for i = 0:5
            est_squareX(i+1) = mycar_posEst(1) + othercars.car{idx}.vel(1)*0.6*i;
            est_squareX(12-i) = est_squareX(i+1);
            
            if est_squareX(i+1) <= 100*10^3
                est_squareY(i+1) = mycar_posEst(2) - 2500;
                est_squareY(12-i) = mycar_posEst(2) + 2500;
                
                mycar_posEst(3) = 0;
            elseif est_squareX(i+1) <= 275*10^3
                
                if i == 0
                    est_squareX(i+1) = mycar_posEst(1);
                    est_squareX(12-i) = mycar_posEst(1);
                    est_squareY(i+1) = mycar_posEst(2);
                    est_squareY(12-i) = mycar_posEst(2);
                    idx_rect = idx_me;
                else
                    for idx_rect = 1:5:201
                        if est_squareX(i+1) - laneChangePath{othercars.car{idx}.goallane, othercars.car{idx}.save.lane_idx}(idx_rect,1) < 0
                            break;
                        end
                    end
                end
                
                
                if idx_rect~=201
                    vx= laneChangePath{othercars.car{idx}.goallane, othercars.car{idx}.save.lane_idx}(idx_rect+1,1)-laneChangePath{othercars.car{idx}.goallane, othercars.car{idx}.save.lane_idx}(idx_rect,1);
                    vy= laneChangePath{othercars.car{idx}.goallane, othercars.car{idx}.save.lane_idx}(idx_rect+1,2)-laneChangePath{othercars.car{idx}.goallane, othercars.car{idx}.save.lane_idx}(idx_rect,2);
                else
                    vx= laneChangePath{othercars.car{idx}.goallane, othercars.car{idx}.save.lane_idx}(idx_rect,1)-laneChangePath{othercars.car{idx}.goallane, othercars.car{idx}.save.lane_idx}(idx_rect-1,1);
                    vy= laneChangePath{othercars.car{idx}.goallane, othercars.car{idx}.save.lane_idx}(idx_rect,2)-laneChangePath{othercars.car{idx}.goallane, othercars.car{idx}.save.lane_idx}(idx_rect-1,2);
                end
                
                %calculate the center point of the red rectangle
                mycar_posEst_det(1) = laneChangePath{othercars.car{idx}.goallane, othercars.car{idx}.save.lane_idx}(idx_rect,1);
                mycar_posEst_det(2) = laneChangePath{othercars.car{idx}.goallane, othercars.car{idx}.save.lane_idx}(idx_rect,2);
                mycar_posEst_det(3) = atan(vy/vx)*180/pi;
                if i == 0
                    mycar_posEst(3) = mycar_posEst_det(3);
                end
                
                %calculate point of the red rectangle from the center point
                left_right_point = get_car_futurepoint(mycar_posEst_det, othercars.car{idx}.W, 5000);
                est_squareX(i+1) = left_right_point(1,1);
                est_squareY(i+1) = left_right_point(1,2);
                est_squareX(12-i) = left_right_point(2,1);
                est_squareY(12-i) = left_right_point(2,2);
                
            else
                est_squareY(i+1) = (77.5-othercars.car{idx}.goallane*5.0)*10^3 - 2500;
                est_squareY(12-i) = (77.5-othercars.car{idx}.goallane*5.0)*10^3 + 2500;
                
                mycar_posEst(3) = 0;
            end
            
        end
        est_squareX(13) = est_squareX(1);
        est_squareY(13) = est_squareY(1);
        %(end) make square of detecting area for IDM(TTCver)----------------------------------
        
        
        % if there are any othercars near index car
        if(~isempty(idx_nearCar))
            nr_cars = length(idx_nearCar);
            for i = 1:nr_cars
                if find(idx_observedcar == idx_nearCar(i))
                    continue
                end
                
                othercars_posEst = update_pos(othercars.car{idx_nearCar(i)}.pos, othercars.car{idx_nearCar(i)}.vel, t);
                in = inpolygon(othercars_posEst(1), othercars_posEst(2), est_squareX, est_squareY);
                
                if any(in, 1)
                    idx_observedcar = [idx_observedcar; idx_nearCar(i)];
                    t_observedcar = [t_observedcar; t];
                    pos_mycarEst = [pos_mycarEst; mycar_posEst];
                    pos_observedcarEst = [pos_observedcarEst; othercars_posEst];
                end
                
            end
        end
        
        % if there is mycar near index car
        if(~isempty(est_mycar))
            
            if find(idx_observedcar == 0)
                continue
            end
            
            othercars_posEst = update_pos(mycar.pos, mycar.vel, t);
            in = inpolygon(othercars_posEst(1), othercars_posEst(2), est_squareX, est_squareY);
            %----
            if any(in, 1) % && othercars.car{idx}.pos(1) < mycar.pos(1)
                idx_observedcar = [idx_observedcar; 0];
                t_observedcar = [t_observedcar; t];
                pos_mycarEst = [pos_mycarEst; mycar_posEst];
                pos_observedcarEst = [pos_observedcarEst; othercars_posEst];
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
    
%     if othercars.car{idx}.pos(1) > othercars.car{i}.pos(1)
%         continue
%     end
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
    
    if othercars.car{i}.pos(1) - othercars.car{idx}.pos(1) < DISTANCE && othercars.car{i}.pos(1) - othercars.car{idx}.pos(1) > 0
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