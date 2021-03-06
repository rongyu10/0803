function [idx_crossingcar, arr_t_othercar, dist_section_mycar, rel_deg_crossingcar, othercar_sidepoint, invadepoint] = detect_crossing_car_1215(othercars, mycar)

idx_nearCar = get_nearCar(mycar, othercars); % 自車から半径50m以内の他車（複数）を特定する
idx_crossingcar = [];
arr_t_mycar = [];
arr_t_othercar = [];
dist_section_mycar = [];
rel_deg_crossingcar = [];
othercar_sidepoint = [];

invadepoint = [];

if isempty(idx_nearCar)
    return
else
    
    nr_cars = length(idx_nearCar);
    
    % 自車から半径50m以内の全ての他車について
    for i = 1:nr_cars
        tmp_invadepoint = [];
        
        % 自車から見た他車の角度（グローバル座標）を求める
        [theta_mycar2other,~] = cart2pol(othercars.car{idx_nearCar(i)}.pos(1) - mycar.pos(1), othercars.car{idx_nearCar(i)}.pos(2) - mycar.pos(2));
       
        % if (mycar.pos(3) - theta_mycar2other*180/pi)*(othercars.car{idx_nearCar(i)}.pos(3) - mycar.pos(3)) >= 0 % 他車が自車方向に向かって走行している場合
        
        % 自車が他車走行領域に侵入する時の侵入点を求める
        tmp_othercar_sidepoint_each = get_sidepoint(othercars.car{idx_nearCar(i)}.pos, mycar.othercars_travel_area_side);
        if mycar.pos(2) > tmp_othercar_sidepoint_each(2,2) + tan(othercars.car{idx_nearCar(i)}.pos(3)*pi/180) * (mycar.pos(1)- tmp_othercar_sidepoint_each(2,1)) % 自車が他車走行予定経路の上側にいる場合
            tmp_othercar_sidepoint = tmp_othercar_sidepoint_each(2,:);
            
            if mycar.pathTranslated(1,2) < tmp_othercar_sidepoint(2) + tan(othercars.car{idx_nearCar(i)}.pos(3)*pi/180) * (mycar.pathTranslated(1,1) - tmp_othercar_sidepoint(1))
                break;
            end
            
            for j = 1:201
                if mycar.pathTranslated(j,2) < tmp_othercar_sidepoint(2) + tan(othercars.car{idx_nearCar(i)}.pos(3)*pi/180) * (mycar.pathTranslated(j,1) - tmp_othercar_sidepoint(1))
%                     if mycar.pathTranslated(j,1) > tmp_othercar_sidepoint(1) % if invade point is front of othercar sidepoint
%                         invade_degree = get_targetdegree(mycar.pathTranslated, j);
%                         tmp_invadepoint = [mycar.pathTranslated(j,1), mycar.pathTranslated(j,2), invade_degree];
%                     end
                    invade_degree = get_targetdegree(mycar.pathTranslated, j);
                    if invade_degree <= othercars.car{idx_nearCar(i)}.pos(3)
                        tmp_invadepoint = [mycar.pathTranslated(j,1), mycar.pathTranslated(j,2), invade_degree];
                        break;
                    end
                end
            end
        elseif mycar.pos(2) < tmp_othercar_sidepoint_each(1,2) + tan(othercars.car{idx_nearCar(i)}.pos(3)*pi/180) * (mycar.pos(1) - tmp_othercar_sidepoint_each(1,1))  % if mycar exists under othercar
            tmp_othercar_sidepoint = tmp_othercar_sidepoint_each(1,:);
            
            if mycar.pathTranslated(1,2) > tmp_othercar_sidepoint(2) + tan(othercars.car{idx_nearCar(i)}.pos(3)*pi/180) * (mycar.pathTranslated(1,1) - tmp_othercar_sidepoint(1))
                break;
            end
            
            for j = 1:201
                if mycar.pathTranslated(j,2) > tmp_othercar_sidepoint(2) + tan(othercars.car{idx_nearCar(i)}.pos(3)*pi/180) * (mycar.pathTranslated(j,1) - tmp_othercar_sidepoint(1))
%                     if mycar.pathTranslated(j,1) > tmp_othercar_sidepoint(1) % if invade point is front of othercar sidepoint
%                         invade_degree = get_targetdegree(mycar.pathTranslated, j);
%                         tmp_invadepoint = [mycar.pathTranslated(j,1), mycar.pathTranslated(j,2), invade_degree];
%                     end
                    invade_degree = get_targetdegree(mycar.pathTranslated, j);
                    if invade_degree >= othercars.car{idx_nearCar(i)}.pos(3)
                        tmp_invadepoint = [mycar.pathTranslated(j,1), mycar.pathTranslated(j,2), invade_degree];
                        break;
                    end
                end
            end
        end
        
        
        if ~isempty(tmp_invadepoint)
            
            if j~=201
                vx= mycar.pathTranslated(j+1,1)-mycar.pathTranslated(j,1);
                vy= mycar.pathTranslated(j+1,2)-mycar.pathTranslated(j,2);
            else
                vx= mycar.pathTranslated(j,1)-mycar.pathTranslated(j-1,1);
                vy= mycar.pathTranslated(j,2)-mycar.pathTranslated(j-1,2);
            end
            
            tmp_rel_deg_crossingcar = abs(othercars.car{idx_nearCar(i)}.pos(3) - atan(vy/vx)*180/pi); % 侵入時の相対角度
            tmp_dist_section_mycar = norm(tmp_invadepoint(1:2) - mycar.pos(1:2)); % 自車の侵入点までの距離
            if tmp_invadepoint(1) > tmp_othercar_sidepoint(1)
                tmp_dist_section_othercar = norm(tmp_invadepoint(1:2) - tmp_othercar_sidepoint);
            else
                tmp_dist_section_othercar = -norm(tmp_invadepoint(1:2) - tmp_othercar_sidepoint);
            end
            tmp_arr_t_mycar = tmp_dist_section_mycar / mycar.vel(1); % 自車の侵入点までの時間
            tmp_arr_t_othercar = tmp_dist_section_othercar / othercars.car{idx_nearCar(i)}.vel(1); % 他車の侵入点までの時間
            
            idx_crossingcar = [idx_crossingcar; idx_nearCar(i)];
            arr_t_othercar = [arr_t_othercar; tmp_arr_t_othercar];
            dist_section_mycar = [dist_section_mycar; tmp_dist_section_mycar];
            rel_deg_crossingcar = [rel_deg_crossingcar; tmp_rel_deg_crossingcar];
            othercar_sidepoint = [othercar_sidepoint; tmp_othercar_sidepoint];
            invadepoint = [invadepoint; tmp_invadepoint];
            
        end
        
        % end
        
        
    end

end

end


function idx_nearCar = get_nearCar(mycar, othercars) % get the number of othercars in front of mycar and close to mycar

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
