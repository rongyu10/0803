function mycar = update_control_mycar_merge_intelligent_OUT(mycar, sim, othercars, idm, laneChangePath, lengthP, FLAG_LANECHANGE)
% UPDATE MY CAR INFORMATION

persistent first_flag first_flag1 first_flag2 h

if isempty(first_flag)
    first_flag = true;
end
if isempty(first_flag1)
    first_flag1 = true;
end
if isempty(first_flag2)
    first_flag2 = true;
end

target_lane = mycar.selectlane;
start_lane = mycar.startlane;


% PARAMETER OF INTELLIGENT DRIVING MODEL---------------------
v0 = idm.v0; % desired velocity
T = idm.T; % Safe time headway
a = idm.a; % maximum acceleration
b = idm.b; %desired deceleration
delta = idm.delta; %acceleration exponent
s0 = idm.s0; % minimum distance
l = idm.l; % vehicle length
%============================================================

if first_flag1 && FLAG_LANECHANGE == 1
    
    % IDM following mycar
    first_flag1 = false;
    
    if mycar.selectlane > mycar.goallane
        mycar.turnSignal = 'right';
    else
        mycar.turnSignal = 'left';
    end
    
    mycar.goallane = mycar.selectlane;
    
    % calculate path after finishing merging
    h.ratioSpeed_tar = lengthP{target_lane, start_lane}/(225*10^3);
    h.afterPath = laneChangePath{target_lane, start_lane};
    
    dist=bsxfun(@hypot,h.afterPath(:,1)-mycar.pos(1), h.afterPath(:,2)-mycar.pos(2));
    [~,idx]=min(dist); % identify the nearest point of the target lane
    h.targetPos = [h.afterPath(idx + 20,1) h.afterPath(idx + 20,2)]; % decide the target point of merging path
    [theta,rho] = cart2pol(h.targetPos(1) - mycar.pos(1), h.targetPos(2) - mycar.pos(2)); % degree and distance from mycar to target point
    if theta - deg2rad(mycar.pos(3)) > 0
        theta = theta - deg2rad(mycar.pos(3));
    else
        theta = deg2rad(mycar.pos(3)) - theta;
    end
    mycar.ctlPt = [mycar.pos(1) mycar.pos(2); mycar.pos(1) + 1/3 * rho * cos(theta) * cos(deg2rad(mycar.pos(3))) mycar.pos(2) + 1/3 * rho * cos(theta) * sin(deg2rad(mycar.pos(3))); h.targetPos(1) - 1/3 * rho * cos(theta) * cos(deg2rad(mycar.pos(3))) h.targetPos(2) - 1/3 * rho * cos(theta) * sin(deg2rad(mycar.pos(3))); h.targetPos(1) h.targetPos(2)];
    [h.mergingPath, h.merginglengthP] = bezierCurve(mycar.ctlPt);
    mycar.mergingPath = h.mergingPath;
    ratioSpeed_mer = h.merginglengthP/rho;
    mycar.vel(1) = mycar.vel(1)*ratioSpeed_mer;
    
    % detect the car number of front/rear car on the target lane 
    for i = 1:nnz(othercars.car_nr(target_lane,:))
        if  othercars.car{othercars.car_nr(target_lane,i)}.pos(1) - h.afterPath(idx,1) < 0
            if i == 1
                mycar.front_nr = 0;
                mycar.rear_nr = othercars.car_nr(target_lane,i);
            else
                mycar.front_nr = othercars.car_nr(target_lane,i - 1);
                mycar.rear_nr = othercars.car_nr(target_lane,i);
            end
            break;
        end
        if i == nnz(othercars.car_nr(target_lane,:))
            mycar.front_nr = othercars.car_nr(target_lane,i);
            mycar.rear_nr = 0;
        end
    end
    fprintf(2, 'LANECHANGE START \n');
end
    
if FLAG_LANECHANGE == 1 % && mycar.rear_nr ~= nnz(othercars.car_nr(target_lane,:))
    
    pos = predict_pos(mycar.pos, mycar.vel, sim.T);
    
    % following afterpath after finished merging
    if mycar.pos(1) < h.targetPos(1)
        [targetDegree,~] = get_tatgetTheta(pos,h.mergingPath);
        
        mycar.pos(3) = targetDegree;
%         if targetDegree - mycar.pos(3) > 5
%             mycar.vel(2) = mycar.vel(2) + 5/sim.T;
%         elseif targetDegree - mycar.pos(3) < -5
%             mycar.vel(2) = mycar.vel(2) - 5/sim.T;
%         else
%             mycar.vel(2) = mycar.vel(2) + (targetDegree - mycar.pos(3))/sim.T;
%         end
        
        % control mycar.vel(1) according to IDM
        A1 = mycar.vel(1)/v0;
        if mycar.front_nr == 0
            A2 = 0;
        else
            A3 = othercars.car{mycar.front_nr}.pos(1) - mycar.pos(1) - l;
            A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{mycar.front_nr}.vel(1))/2/sqrt(a*b))/A3;
        end

        mycar.vel(1) = mycar.vel(1) + a*(1 - A1^delta - A2^2)*sim.T;
        
    else
        if first_flag2
            first_flag2 = false;
            mycar.vel(1) = mycar.vel(1)*h.ratioSpeed_tar;
            mycar.turnSignal = '';
            fprintf(2, 'LANECHANGE FINISHED \n');
        end
        
        [targetDegree,~] = get_tatgetTheta(pos,h.afterPath);
        mycar.pos(3) = targetDegree;
        
%         if targetDegree - mycar.pos(3) > 5
%             mycar.vel(2) = mycar.vel(2) + 5/sim.T;
%         elseif targetDegree - mycar.pos(3) < -5
%             mycar.vel(2) = mycar.vel(2) - 5/sim.T;
%         else
%             mycar.vel(2) = mycar.vel(2) + (targetDegree - mycar.pos(3))/sim.T;
%         end
        
        % control mycar.vel(1) according to IDM
        A3 = othercars.car{mycar.front_nr}.pos(1) - mycar.pos(1) - l;
        A1 = mycar.vel(1)/v0;
        A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{mycar.front_nr}.vel(1))/2/sqrt(a*b))/A3;
        
        mycar.vel(1) = mycar.vel(1) + a*(1 - A1^delta - A2^2)*sim.T;
        
    end
    
    if mycar.vel(1) < 0
        mycar.vel(1) = 0;
    end
    
elseif first_flag && mycar.pos(1) > 45*10^3 % if mycar enters the plaza
    first_flag = false;
    ratioSpeed_cur = lengthP{mycar.goallane, mycar.startlane}/(225*10^3);
    mycar.vel(1) = mycar.vel(1)*ratioSpeed_cur;
    
elseif FLAG_LANECHANGE == 0
    %---- Control angular velocity: vel(2) --------
    pos = predict_pos(mycar.pos, mycar.vel, sim.T);
    [targetDegree,~] = get_tatgetTheta(pos,laneChangePath{mycar.goallane, mycar.startlane});
    if mycar.pos(1) < 45*10^3
        targetDegree = 0;
    end
    mycar.pos(3) = targetDegree;
    
    if mycar.pos(1) > 45*10^3 % if mycar enters the plaza
        % control mycar.vel(1) according to IDM
        A1 = mycar.vel(1)/v0;
        if mycar.front_nr == 0
            A2 = 0;
        else
            A3 = othercars.car{mycar.front_nr}.pos(1) - mycar.pos(1) - l;
            A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{mycar.front_nr}.vel(1))/2/sqrt(a*b))/A3;
        end
        
        mycar.vel(1) = mycar.vel(1) + a*(1 - A1^delta - A2^2)*sim.T;
    end
    
%         if targetDegree - mycar.pos(3) > 5
%             mycar.vel(2) = mycar.vel(2) + 5/sim.T;
%         elseif targetDegree - mycar.pos(3) < -5
%             mycar.vel(2) = mycar.vel(2) - 5/sim.T;
%         else
%             mycar.vel(2) = mycar.vel(2) + (targetDegree - mycar.pos(3))/sim.T;
%         end
end


mycar.pos = update_pos(mycar.pos, mycar.vel, sim.T);
mycar.bd  = get_carshape(mycar.pos, mycar.W, mycar.H);


%mycar = update_rfs(mycar, othercars);

end

function [targetDegree, idx] = get_tatgetTheta(pos,path)

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




function mycar = update_rfs(mycar, othercars)
mycar.r = 0;
% EMULATE RFS
D2R    = pi/180;
nr_rfs = mycar.nr_rfs;
result = mycar.rfs_dist*ones(1, nr_rfs);
for i = 1:mycar.nr_rfs
    % FOR ALL RANGEFINDER BEAMS
    rfs_deg = mycar.pos(3) + mycar.rfs_degs(i);
    rfs_rad = rfs_deg * D2R;
    rfs_fr  = mycar.pos(1:2);
    rfs_to  = mycar.pos(1:2) ...
        + (mycar.r+mycar.rfs_dist)*[cos(rfs_rad) sin(rfs_rad)];
    min_dist2obs = mycar.rfs_dist;
    obs_boundary = zeros(2, 1E3);
    nr_obs_b = 0;
    for j = 1:othercars.n
        % FOR ALL OBSTACLES
        curr_obs_boundary = othercars.car{j}.bd;
        curr_obs_boundary = [curr_obs_boundary' [NaN ; NaN]];
        nr_curr_obs_b = size(curr_obs_boundary, 2);
        obs_boundary(:, nr_obs_b+1:nr_obs_b + nr_curr_obs_b) = curr_obs_boundary;
        nr_obs_b = nr_obs_b + nr_curr_obs_b;
    end
    obs_boundary = obs_boundary(:, 1:nr_obs_b);
    
    % Use InterX
    int_xy = InterX([rfs_fr(1) rfs_to(1) ; rfs_fr(2), rfs_to(2)], obs_boundary);
    if ~isempty(int_xy)
        xi = int_xy(1, :);
        yi = int_xy(2, :);
        for k = 1:length(xi)
            dist = norm([ mycar.pos(1)-xi(k), mycar.pos(2)-yi(k) ]);
            dist = dist - mycar.r;
            if dist < min_dist2obs
                min_dist2obs = dist;
            end
        end
    end
    result(i) = min_dist2obs;
end
mycar.rfs_dists = result;
end
