function [idx_observedcar, t_observedcar, pos_mycarEst, pos_observedcarEst, idx_observing_mycar, angle_observing_mycar, mycarpos_observing_mycar, t_observing_mycar] = is_carcrashed_orFollow_formycar_TTC_forwardrect(othercars, mycar, laneChangePath, FLAG_OTHERCAR_INTENTION_EST)

idx_nearCar = get_nearCar(mycar, othercars);
% idx_nearCar = get_frontCar(mycar, othercars);
idx_observedcar = []; % 0:mycar 1~:number of othercar
t_observedcar = [];
pos_mycarEst = [];
pos_observedcarEst = [];
idx_observing_mycar = [];
angle_observing_mycar = [];
mycarpos_observing_mycar = [];
t_observing_mycar = [];

if isempty(idx_nearCar)
    return
else
    clk_TTC = clock;
    
    for t = 0:mycar.step_TTC:mycar.time_TTC
        
        mycar_posEst(1) = mycar.pos(1) + mycar.vel(1)*t;
        
        if mycar_posEst(1) <= 100*10^3
            mycar_posEst(2) = mycar.pos(2);
            mycar_posEst(3) = 0;
        elseif mycar_posEst(1) <= 275*10^3
            %nData = size(laneChangePath{mycar.goallane, mycar.save.lane_idx},1);
            for idx_me = 1:201
                if mycar_posEst(1) - 100*10^3 - 175/200*(idx_me - 1)*10^3 < 0
                    mycar_posEst(1) = laneChangePath{mycar.goallane, mycar.save.lane_idx}(idx_me,1);
                    mycar_posEst(2) = laneChangePath{mycar.goallane, mycar.save.lane_idx}(idx_me,2);
                    
                    if idx_me~=201
                        vx= laneChangePath{mycar.goallane, mycar.save.lane_idx}(idx_me+1,1)-laneChangePath{mycar.goallane, mycar.save.lane_idx}(idx_me,1);
                        vy= laneChangePath{mycar.goallane, mycar.save.lane_idx}(idx_me+1,2)-laneChangePath{mycar.goallane, mycar.save.lane_idx}(idx_me,2);
                    else
                        vx= laneChangePath{mycar.goallane, mycar.save.lane_idx}(idx_me,1)-laneChangePath{mycar.goallane, mycar.save.lane_idx}(idx_me-1,1);
                        vy= laneChangePath{mycar.goallane, mycar.save.lane_idx}(idx_me,2)-laneChangePath{mycar.goallane, mycar.save.lane_idx}(idx_me-1,2);
                    end
                    mycar_posEst(3) = atan(vy/vx)*180/pi;
                    break;
                end
            end
        else
            mycar_posEst(2) = (77.5-mycar.goallane*5.0)*10^3;
            mycar_posEst(3) = 0;
        end
        
        [est_squareX, est_squareY] = make_detecting_rectangle(mycar, mycar_posEst, mycar.vel, laneChangePath, mycar.detect_rect_forwardtime, mycar.detect_rect_sidewidth);
        
        
        nr_cars = length(idx_nearCar);
        for i = 1:nr_cars
            if ~isempty(find(idx_observedcar == idx_nearCar(i), 1)) || ~isempty(find(idx_observing_mycar == idx_nearCar(i), 1)) % if the target index's collision is already detected
                continue
            end
            
            othercars_posEst = update_pos(othercars.car{idx_nearCar(i)}.pos, othercars.car{idx_nearCar(i)}.vel, t);
            
            % detect the index(and so on) of othercars in mycar's detecting area---------------
            in = inpolygon(othercars_posEst(1), othercars_posEst(2), est_squareX, est_squareY);
            
            if any(in, 1)
                idx_observedcar = [idx_observedcar; idx_nearCar(i)];
                t_observedcar = [t_observedcar; t];
                pos_mycarEst = [pos_mycarEst; mycar_posEst];
                pos_observedcarEst = [pos_observedcarEst; othercars_posEst];
            end
            %------------------------------
            
            if FLAG_OTHERCAR_INTENTION_EST
                % detect the index(and so on) of othercars which detect mycar in its detecting area---------------
                for j = 0:1
                    
                    Pos_est = update_pos(othercars_posEst, othercars.car{idx_nearCar(i)}.vel, j*3.0);
                    
                    left_right_point = get_car_futurepoint(Pos_est, mycar.W, 5000);
                    othercar_est_squareX(j+1) = left_right_point(1,1);
                    othercar_est_squareY(j+1) = left_right_point(1,2);
                    othercar_est_squareX(4-j) = left_right_point(2,1);
                    othercar_est_squareY(4-j) = left_right_point(2,2);
                    
                end
                othercar_est_squareX(5) = othercar_est_squareX(1);
                othercar_est_squareY(5) = othercar_est_squareY(1);
                in = inpolygon(mycar_posEst(1), mycar_posEst(2), othercar_est_squareX, othercar_est_squareY);
                
                if any(in, 1)
                    idx_observing_mycar = [idx_observing_mycar; idx_nearCar(i)];
                    angle_observing_mycar = [angle_observing_mycar; abs(mycar_posEst(3) - othercars_posEst(3))];
                    mycarpos_observing_mycar = [mycarpos_observing_mycar; mycar_posEst];
                    t_observing_mycar = [t_observing_mycar; t];
                end
                %------------------------------
            end
        end
        
    end
    ms_TTC  = etime(clock, clk_TTC)*1000;
    %fprintf(2, 'TTC calculating = [%d]msec\n', ms_TTC);
end

end


function idx_nearCar = get_nearCar(mycar, othercars) % get the number of othercars in front of mycar and close to mycar

% DISTANCE = mycar.vel(1)*3;     % distance running in 3 seconds
DISTANCE = 50000;
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

function idx_nearCar = get_frontCar(mycar, othercars) % get the number of othercars in front of mycar and close to mycar

DISTANCE = mycar.vel(1)*3;     % distance running in 3 seconds

mycar_pos = mycar.pos(1:2);
nr_cars = othercars.n;

idx_nearCar =[];
for i=1:nr_cars
    
%     if mycar.pos(1) > othercars.car{i}.pos(1)
%         continue
%     end
    
    if abs(mycar.pos(1)-othercars.car{i}.pos(1)) < DISTANCE
        
        
        [theta_mycar2other,~] = cart2pol(mycar.pos(1) - othercars.car{i}.pos(1), mycar.pos(2) - othercars.car{i}.pos(2));
        if abs(mycar.pos(3) - theta_mycar2other*180/pi) > 90
            fprintf(1, 'car[%d] is back of mycar [%d]\n', i, mycar.pos(3) - theta_mycar2other*180/pi);
            continue
        end
        
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
