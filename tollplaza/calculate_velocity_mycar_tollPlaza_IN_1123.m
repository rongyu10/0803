function mycar = calculate_velocity_mycar_tollPlaza_IN_1123(mycar, sim, othercars, idm, laneChangePath)


idx_precedingcar = [];
mycar.acceleration = [];


    % if entering the plaza
if mycar.flgPlaza == 0 && mycar.pos(1) > 100*10^3
    mycar.flgPlaza = 1;
    
    mycar.pathTranslated = laneChangePath{mycar.goallane, mycar.save.lane_idx};
end

if mycar.flgPlaza == 0 % before entering plaza
    idm.v0 = 15000;
    
elseif mycar.flgPlaza == 1 % after entering plaza
    pos = predict_pos(mycar.pos, mycar.vel, sim.T);
    mycar.targetDegree = get_tatgetTheta(pos,mycar.pathTranslated);
    mycar.vel(2) = (mycar.targetDegree - mycar.pos(3))/sim.T;
    
    if mycar.pos(1) < 187.5*10^3
        idm.v0 = 15000;

    elseif mycar.pos(1) < 275*10^3
        idm.v0 = 12500;
        
    elseif mycar.pos(1) < 320*10^3
        idm.v0 = 10000;
    end
    
end

[mycar.squareX, mycar.squareY] = make_detecting_rectangle_lengthfix(mycar, mycar.pos, laneChangePath, mycar.detect_rect_length, mycar.detect_rect_sidewidth);


% estimate crashing othercar and identify approaching othercar in front of mycar----------------------
[idx_precedingcar, rel_degree_precedingcar] = detect_preceding_car(othercars, mycar);



if ~isempty(idx_precedingcar)
    mycar.acceleration = calculate_acceleration_IDM_following(mycar, othercars.car{idx_precedingcar}, othercars.car{idx_precedingcar}.pos, idm, rel_degree_precedingcar);
else
    A1 = mycar.vel(1)/idm.v0;
    mycar.acceleration = idm.a*(1 - A1^idm.delta);
end


mycar.vel(1) = mycar.vel(1) + mycar.acceleration*sim.T;


% --regulate acceleration and velocity ---------------------
if mycar.acceleration > 2940
    mycar.acceleration = 2940;
elseif mycar.acceleration < -2940
    mycar.acceleration = -2940;
end

if ~isempty(idx_precedingcar)
    if mycar.acceleration < -1960
        fprintf(2, 'mycar([%d, %d]) decelerate([%d]) to car [%d] (reldegree = [%d])\n', mycar.pos(1), mycar.pos(2), mycar.acceleration, idx_precedingcar, rel_degree_precedingcar);
    else
        fprintf(1, 'mycar([%d, %d]) decelerate([%d]) to car [%d] (reldegree = [%d])\n', mycar.pos(1), mycar.pos(2), mycar.acceleration, idx_precedingcar, rel_degree_precedingcar);
    end
end

if mycar.vel(1) < 0 && mycar.pos(1) < 320 * 10^3
    mycar.vel(1) = 0;
end
% -----------------------------------------------------------

end

function targetDegree = get_tatgetTheta(pos,path)

nData = size(path,1);

dist=bsxfun(@hypot,path(:,1)-pos(1),path(:,2)-pos(2));
[~,idx]=min(dist);

if idx~=nData
    vx= path(idx+1,1)-path(idx,1);
    vy= path(idx+1,2)-path(idx,2);
else
    vx= path(idx,1)-path(idx-1,1);
    vy= path(idx,2)-path(idx-1,2);
end

targetDegree =atan(vy/vx)*180/pi;

end

function pos = predict_pos(pos, vel, T)

c = cos(pos(3)*pi/180);
s = sin(pos(3)*pi/180);
pos(1:2) = pos(1:2) + vel(1)*T*[c s];
pos(3) = pos(3) + vel(2)*T;

% DETERMINE DEGREE TO LIE BETWEEN -180~180
while pos(3) > 180
    pos(3) = pos(3) - 360;
end
while pos(3) < -180
    pos(3) = pos(3) + 360;
end

end