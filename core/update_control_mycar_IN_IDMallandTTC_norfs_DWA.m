function [mycar, othercars] = update_control_mycar_IN_IDMallandTTC_norfs_DWA(mycar, sim, othercars, idm, laneChangePath)

% PARAMETER OF INTELLIGENT DRIVING MODEL---------------------
v0 = idm.v0; % desired velocity
T = idm.T; % Safe time headway
a = idm.a; % maximum acceleration
b = idm.b; %desired deceleration
delta = idm.delta; %acceleration exponent
s0 = idm.s0; % minimum distance
l = idm.l; % vehicle length
%============================================================

% PARAMETER OF TTC-------------------------------------------
time_TTC = 5.0;
step_TTC = 0.1;

idx_crashcar = [];


if mycar.flgPlaza == 0 % before entering plaza
    
    v0 = 15000;
    mycar.acceleration = 0;
    
    if mycar.pos(1) > 0
        [idx_crashcar, t, mycar_posEst, idx_frontCar] = is_carcrashed_formycar_TTC_verIDM_inpol4_mycar(othercars, time_TTC, step_TTC, mycar);
        
        % identify index of nearest othercar in the observing box 
        c = cos(mycar.pos(3)*pi/180);
        s = sin(mycar.pos(3)*pi/180);
        squareX = [mycar.pos(1)+2500*s+6000*c mycar.pos(1)+2500*s+30000*c mycar.pos(1)-2500*s+30000*c mycar.pos(1)-2500*s+6000*c mycar.pos(1)+2500*s+6000*c];
        squareY = [mycar.pos(2)-2500*c+6000*s mycar.pos(2)-2500*c+30000*s mycar.pos(2)+2500*c+30000*s mycar.pos(2)+2500*c+6000*s mycar.pos(2)-2500*c+6000*s];
        
        nr_frontCar = length(idx_frontCar);
        idx_mindist = [];
        dist_min = 30000;
        for i = 1:nr_frontCar
            in = inpolygon(othercars.car{idx_frontCar(i)}.pos(1), othercars.car{idx_frontCar(i)}.pos(2), squareX, squareY);
            if in == 1
                if norm(mycar.pos(1:2) - othercars.car{idx_frontCar(i)}.pos(1:2)) < dist_min
                    dist_min = norm(mycar.pos(1:2) - othercars.car{idx_frontCar(i)}.pos(1:2));
                    idx_mindist = idx_frontCar(i);
                end
            end
        end
        
        if ~isempty(idx_crashcar) % && idx_mindist ~= idx_crashcar
            
            fprintf(1, 'after [%d] seconds, mycar and [%d](%d, %d) collide\n', t, idx_crashcar, othercars.car{idx_crashcar}.pos(1), othercars.car{idx_crashcar}.pos(2));
            
            A3 = norm(mycar_posEst(1:2) - mycar.pos(1:2));
            if A3 < 4000
                A3 = 4000;
            end
            A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{idx_crashcar}.vel(1))/2/sqrt(a*b))/A3;
            A1 = mycar.vel(1)/v0;
            mycar.acceleration = a*(1 - A1^delta - A2^2);
            
            if ~isempty(idx_mindist) % IDM following frontcar in the observing box
                fprintf(1, 'Following car [%d](%d, %d) by IDM\n',idx_mindist, othercars.car{idx_mindist}.pos(1), othercars.car{idx_mindist}.pos(2));
                A3 = norm(othercars.car{idx_mindist}.pos(1:2) - mycar.pos(1:2)) - l;
                A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{idx_mindist}.vel(1))/2/sqrt(a*b))/A3;
            else
                A2 = 0;
            end
            A1 = mycar.vel(1)/v0;
            
            if a*(1 - A1^delta - A2^2) < mycar.acceleration
                mycar.acceleration = a*(1 - A1^delta - A2^2);
                fprintf(1, 'calculated by IDM is larger deceleration\n');
            end
            
        else
            if ~isempty(idx_mindist) % IDM following frontcar
                fprintf(1, 'Following car [%d](%d, %d) by IDM\n',idx_mindist, othercars.car{idx_mindist}.pos(1), othercars.car{idx_mindist}.pos(2));
                A3 = norm(othercars.car{idx_mindist}.pos(1:2) - mycar.pos(1:2)) - l;
                A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{idx_mindist}.vel(1))/2/sqrt(a*b))/A3;
            else
                A2 = 0;
            end
            A1 = mycar.vel(1)/v0;
            mycar.acceleration = a*(1 - A1^delta - A2^2);
            
        end
        
        if mycar.acceleration < -9800
            fprintf(2, 'mycar(%d, %d) acceleration = [%4d] \n', mycar.pos(1), mycar.pos(2), mycar.acceleration);
            %mycar.acceleration = -9800;
        else
            fprintf(1, 'mycar(%d, %d) acceleration = [%4d] \n', mycar.pos(1), mycar.pos(2), mycar.acceleration);
        end
        
    end
    
    % if entering the plaza
    if mycar.pos(1) > 100*10^3 && mycar.pos(1) < 275*10^3
        mycar.flgPlaza = 1;
        
        mycar.pathTranslated = laneChangePath{mycar.selectlane, mycar.save.lane_idx};
    end
    
elseif mycar.flgPlaza == 1 % after entering plaza
    
    pos = predict_pos(mycar.pos, mycar.vel, sim.T);
    targetDegree = get_tatgetTheta(pos,mycar.pathTranslated);
    mycar.pos(3) = targetDegree;
    
    if mycar.pos(1) < 187.5*10^3
        v0 = 12500;
        [idx_crashcar, t, mycar_posEst, idx_frontCar] = is_carcrashed_formycar_TTC_verIDM_inpol4_mycar(othercars, time_TTC, step_TTC, mycar);
        
        c = cos(mycar.pos(3)*pi/180);
        s = sin(mycar.pos(3)*pi/180);
        squareX = [mycar.pos(1)+2500*s+6000*c mycar.pos(1)+2500*s+30000*c mycar.pos(1)-2500*s+30000*c mycar.pos(1)-2500*s+6000*c mycar.pos(1)+2500*s+6000*c];
        squareY = [mycar.pos(2)-2500*c+6000*s mycar.pos(2)-2500*c+30000*s mycar.pos(2)+2500*c+30000*s mycar.pos(2)+2500*c+6000*s mycar.pos(2)-2500*c+6000*s];
        
        nr_frontCar = length(idx_frontCar);
        idx_mindist = [];
        dist_min = 30000;
        for i = 1:nr_frontCar
            in = inpolygon(othercars.car{idx_frontCar(i)}.pos(1), othercars.car{idx_frontCar(i)}.pos(2), squareX, squareY);
            if in == 1
                if norm(mycar.pos(1:2) - othercars.car{idx_frontCar(i)}.pos(1:2)) < dist_min
                    dist_min = norm(mycar.pos(1:2) - othercars.car{idx_frontCar(i)}.pos(1:2));
                    idx_mindist = idx_frontCar(i);
                end
            end
        end
        
        if (~isempty(idx_crashcar)) % && (idx_mindist ~= idx_crashcar)
            
            fprintf(1, 'after [%d] seconds, mycar and [%d](%d, %d) collide\n', t, idx_crashcar, othercars.car{idx_crashcar}.pos(1), othercars.car{idx_crashcar}.pos(2));
            
            A3 = norm(mycar_posEst(1:2) - mycar.pos(1:2));
            if A3 < 4000
                A3 = 4000;
            end
            A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{idx_crashcar}.vel(1))/2/sqrt(a*b))/A3;
            A1 = mycar.vel(1)/v0;
            mycar.acceleration = a*(1 - A1^delta - A2^2);
            
            
            if ~isempty(idx_mindist) % IDM following frontcar
                fprintf(1, 'Following car [%d](%d, %d) by IDM\n',idx_mindist, othercars.car{idx_mindist}.pos(1), othercars.car{idx_mindist}.pos(2));
                A3 = norm(othercars.car{idx_mindist}.pos(1:2) - mycar.pos(1:2)) - l;
                A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{idx_mindist}.vel(1))/2/sqrt(a*b))/A3;
            else
                A2 = 0;
            end
            A1 = mycar.vel(1)/v0;
            
            if a*(1 - A1^delta - A2^2) < mycar.acceleration
                mycar.acceleration = a*(1 - A1^delta - A2^2);
                fprintf(1, 'calculated by IDM is larger deceleration\n');
            end
            
        else
            if ~isempty(idx_mindist) % IDM following frontcar
                fprintf(1, 'Following car [%d](%d, %d) by IDM\n',idx_mindist, othercars.car{idx_mindist}.pos(1), othercars.car{idx_mindist}.pos(2));
                A3 = norm(othercars.car{idx_mindist}.pos(1:2) - mycar.pos(1:2)) - l;
                A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{idx_mindist}.vel(1))/2/sqrt(a*b))/A3;
            else
                A2 = 0;
            end
            A1 = mycar.vel(1)/v0;
            mycar.acceleration = a*(1 - A1^delta - A2^2);
            
        end
        
        if mycar.acceleration < -9800
            fprintf(2, 'mycar(%d, %d) acceleration = [%4d] \n', mycar.pos(1), mycar.pos(2), mycar.acceleration);
            %mycar.acceleration = -9800;
        else
            fprintf(1, 'mycar(%d, %d) acceleration = [%4d] \n', mycar.pos(1), mycar.pos(2), mycar.acceleration);
        end
        
    elseif mycar.pos(1) < 275*10^3
        v0 = 10000;
        if mycar.flgIDM == 0
            
            % set the index number among the same target lane
            if isempty(find(othercars.car_nr(mycar.selectlane,:), 1, 'last')) % if there is no othercars heading for same lane
                %mycar.front_nr = 0;
                idx_nr = 1;
            else
                %mycar.front_nr = othercars.car_nr(mycar.selectlane, find(othercars.car_nr(mycar.selectlane,:), 1, 'last'));
                idx_nr = find(othercars.car_nr(mycar.selectlane,:), 1, 'last') + 1;
            end
            othercars.car_nr(mycar.selectlane, idx_nr) = -1;
            mycar.flgIDM = 1;
        end
        
        [idx_crashcar, t, mycar_posEst, idx_frontCar] = is_carcrashed_formycar_TTC_verIDM_inpol4_mycar(othercars, time_TTC, step_TTC, mycar);
        
        c = cos(mycar.pos(3)*pi/180);
        s = sin(mycar.pos(3)*pi/180);
        squareX = [mycar.pos(1)+2500*s+6000*c mycar.pos(1)+2500*s+30000*c mycar.pos(1)-2500*s+30000*c mycar.pos(1)-2500*s+6000*c mycar.pos(1)+2500*s+6000*c];
        squareY = [mycar.pos(2)-2500*c+6000*s mycar.pos(2)-2500*c+30000*s mycar.pos(2)+2500*c+30000*s mycar.pos(2)+2500*c+6000*s mycar.pos(2)-2500*c+6000*s];
        
        nr_frontCar = length(idx_frontCar);
        idx_mindist = [];
        dist_min = 30000;
        for i = 1:nr_frontCar
            in = inpolygon(othercars.car{idx_frontCar(i)}.pos(1), othercars.car{idx_frontCar(i)}.pos(2), squareX, squareY);
            if in == 1
                if norm(mycar.pos(1:2) - othercars.car{idx_frontCar(i)}.pos(1:2)) < dist_min
                    dist_min = norm(mycar.pos(1:2) - othercars.car{idx_frontCar(i)}.pos(1:2));
                    idx_mindist = idx_frontCar(i);
                end
            end
        end
        
        if (~isempty(idx_crashcar)) % && (idx_mindist ~= idx_crashcar)
            
            fprintf(1, 'after [%d] seconds, mycar and [%d](%d, %d) collide\n', t, idx_crashcar, othercars.car{idx_crashcar}.pos(1), othercars.car{idx_crashcar}.pos(2));
            
            A3 = norm(mycar_posEst(1:2) - mycar.pos(1:2));
            if A3 < 4000
                A3 = 4000;
            end
            A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{idx_crashcar}.vel(1))/2/sqrt(a*b))/A3;
            A1 = mycar.vel(1)/v0;
            mycar.acceleration = a*(1 - A1^delta - A2^2);
            
            if ~isempty(idx_mindist) % IDM following frontcar
                fprintf(1, 'Following car [%d](%d, %d) by IDM\n',idx_mindist, othercars.car{idx_mindist}.pos(1), othercars.car{idx_mindist}.pos(2));
                A3 = norm(othercars.car{idx_mindist}.pos(1:2) - mycar.pos(1:2)) - l;
                A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{idx_mindist}.vel(1))/2/sqrt(a*b))/A3;
            else
                A2 = 0;
            end
            A1 = mycar.vel(1)/v0;
            
            if a*(1 - A1^delta - A2^2) < mycar.acceleration
                mycar.acceleration = a*(1 - A1^delta - A2^2);
                fprintf(1, 'calculated by IDM is larger deceleration\n');
            end
            
        else
            if ~isempty(idx_mindist) % IDM following frontcar
                fprintf(1, 'Following car [%d](%d, %d) by IDM\n',idx_mindist, othercars.car{idx_mindist}.pos(1), othercars.car{idx_mindist}.pos(2));
                A3 = norm(othercars.car{idx_mindist}.pos(1:2) - mycar.pos(1:2)) - l;
                A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{idx_mindist}.vel(1))/2/sqrt(a*b))/A3;
            else
                A2 = 0;
            end
            A1 = mycar.vel(1)/v0;
            mycar.acceleration = a*(1 - A1^delta - A2^2);
            
        end
        
        if mycar.acceleration < -9800
            fprintf(2, 'mycar(%d, %d) acceleration = [%4d] \n', mycar.pos(1), mycar.pos(2), mycar.acceleration);
            %mycar.acceleration = -9800;
        else
            fprintf(1, 'mycar(%d, %d) acceleration = [%4d] \n', mycar.pos(1), mycar.pos(2), mycar.acceleration);
        end
        
    elseif mycar.pos(1) < 320*10^3
        
        [~, ~, ~, idx_frontCar] = is_carcrashed_formycar_TTC_verIDM_inpol4_mycar(othercars, time_TTC, step_TTC, mycar);
        
        c = cos(mycar.pos(3)*pi/180);
        s = sin(mycar.pos(3)*pi/180);
        squareX = [mycar.pos(1)+2500*s+6000*c mycar.pos(1)+2500*s+30000*c mycar.pos(1)-2500*s+30000*c mycar.pos(1)-2500*s+6000*c mycar.pos(1)+2500*s+6000*c];
        squareY = [mycar.pos(2)-2500*c+6000*s mycar.pos(2)-2500*c+30000*s mycar.pos(2)+2500*c+30000*s mycar.pos(2)+2500*c+6000*s mycar.pos(2)-2500*c+6000*s];
        
        nr_frontCar = length(idx_frontCar);
        idx_mindist = [];
        dist_min = 30000;
        for i = 1:nr_frontCar
            in = inpolygon(othercars.car{idx_frontCar(i)}.pos(1), othercars.car{idx_frontCar(i)}.pos(2), squareX, squareY);
            if in == 1
                if norm(mycar.pos(1:2) - othercars.car{idx_frontCar(i)}.pos(1:2)) < dist_min
                    dist_min = norm(mycar.pos(1:2) - othercars.car{idx_frontCar(i)}.pos(1:2));
                    idx_mindist = idx_frontCar(i);
                end
            end
        end
        
        if ~isempty(idx_mindist) % IDM following frontcar
            fprintf(1, 'Following car [%d](%d, %d) by IDM\n',idx_mindist, othercars.car{idx_mindist}.pos(1), othercars.car{idx_mindist}.pos(2));
            A3 = norm(othercars.car{idx_mindist}.pos(1:2) - mycar.pos(1:2)) - l;
            A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{idx_mindist}.vel(1))/2/sqrt(a*b))/A3;
        else
            A2 = 0;
        end
        A1 = mycar.vel(1)/v0;
        mycar.acceleration = a*(1 - A1^delta - A2^2);
        
        if mycar.acceleration < -9800
            fprintf(2, 'mycar(%d, %d) acceleration = [%4d] \n', mycar.pos(1), mycar.pos(2), mycar.acceleration);
            %mycar.acceleration = -9800;
        else
            fprintf(1, 'mycar(%d, %d) acceleration = [%4d] \n', mycar.pos(1), mycar.pos(2), mycar.acceleration);
        end
    end
end

mycar.vel(1) = mycar.vel(1) + mycar.acceleration*sim.T;

% decelerate in the toll lane
if mycar.pos(1) > 275*10^3 && mycar.vel(1) > 5000
    mycar.vel(1) = mycar.vel(1) - 3000 * sim.T; % decelerate if exceeds 18km/h in the tolllane
end

% control minimum velocity
if mycar.vel(1) < 0 && mycar.pos(1) < 320 * 10^3
    mycar.vel(1) = 0;
end

% UPDATE MY CAR INFORMATION
mycar.pos = update_pos(mycar.pos, mycar.vel, sim.T);
mycar.bd  = get_carshape(mycar.pos, mycar.W, mycar.H);

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

function [trajX, trajY] = get_trajData(traj)

traj(:,end+1) = NaN;
%-----
ndata = size(traj,1);
indx  = 1:5:ndata;
indy  = 2:5:ndata;
tmpX = traj(indx,:);
tmpY = traj(indy,:);
%----
trajX = reshape(tmpX',1,[]);
trajY = reshape(tmpY',1,[]);
%size(trajX)

end

function [u,trajDB,evalDB]=DynamicWindowApproach(x,model,goal,evalParam,ob,R,dt)
%DWAによる入力値の計算をする関数

%Dynamic Window[vmin,vmax,ωmin,ωmax]の作成
Vr=CalcDynamicWindow(x,model,dt);
%評価関数の計算
[evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam,dt);

if isempty(evalDB)
    disp('no path to goal!!');
    u=[0;0];return;
end

%各評価関数の正規化
evalDB=NormalizeEval(evalDB);

%最終評価値の計算
feval=[];
for id=1:length(evalDB(:,1))
    feval=[feval;evalParam(1:3)*evalDB(id,3:5)'];
end
evalDB=[evalDB feval];

[maxv,ind]=max(feval);%最も評価値が大きい入力値のインデックスを計算
u=evalDB(ind,1:2)';%評価値が高い入力値を返す
end

function [u,trajDB,evalDB]=TimeVaryingDynamicWindowApproach(x,pedestrians,model,goal,evalParam,ob,R,dt)
%DWAによる入力値の計算をする関数

%Dynamic Window[vmin,vmax,ωmin,ωmax]の作成
Vr=CalcDynamicWindow(x,model,dt);
%評価関数の計算
[evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam,dt);

if isempty(evalDB)
    disp('no path to goal!!');
    u=[0;0];return;
end

%各評価関数の正規化
evalDB=NormalizeEval(evalDB);

%最終評価値の計算
feval=[];
for id=1:length(evalDB(:,1))
    feval=[feval;evalParam(1:3)*evalDB(id,3:5)'];
end
evalDB=[evalDB feval];

%-------------------------
 [trajDB,evalDB]=TimeVaringMethod(x,pedestrians,evalParam,trajDB,evalDB);
 if isempty(evalDB)
    u = [0,0]';
    return
 end
 feval = evalDB(:,end);
%-------------------------


[maxv,ind]=max(feval);%最も評価値が大きい入力値のインデックスを計算
u=evalDB(ind,1:2)';%評価値が高い入力値を返す
end

function [trajDB2,evalDB2]=TimeVaringMethod(x,pedestrians,evalParam,trajDB,evalDB)
% TimeVaringMethod

%--- get id of near people --------
pos = x(1:2)';
if pedestrians.n > 0
   idx_people = [];
   dist_judge = 10*10^3;
   for i = 1:pedestrians.n
        pos_pedestrian = pedestrians.person{i}.pos(1:2);
        if norm(pos_pedestrian-pos,2) < dist_judge
            idx_people = [idx_people,i];
        end
   end
else
   trajDB2 = trajDB;
   evalDB2 = evalDB;
   return
end

if isempty(idx_people)
   trajDB2 = trajDB;
   evalDB2 = evalDB;
   return
end
%--------------------------------
nbox  = length(idx_people);
for i = 1:nbox
    idx = idx_people(i);
    box{i} = get_searchBOX(pedestrians,idx,evalParam);
end

%----- searching trajectory intersecting with future pedestirans positions
ntraj   = size(evalDB,1);
idx_trj = [];
for i = 1:ntraj
    irow = 1 + (i-1)*5;
    traj = trajDB(irow:irow+1,:);
    flag = 0;
    for j = 1:nbox
        int_xy = InterX(traj, box{j});
        if ~isempty(int_xy) % when intersecting
            flag = 1;
        end
    end
    if flag==0
       idx_trj = [idx_trj, i];
    end
end
%---------------------------------------------------------------------

if isempty(idx_trj)
   trajDB2 = [];
   evalDB2 = [];
   return
end

if length(idx_trj)==ntraj
   trajDB2 = trajDB;
   evalDB2 = evalDB;
   return
end

itraj  = length(idx_trj)*5;
jtraj  = size(trajDB,2);
trajDB2 = zeros(itraj,jtraj);
for i = 1:length(idx_trj)
    idx  = idx_trj(i);
    irow     =  1 + (idx-1)*5;
    irow_new = 1+(i-1)*5;
    trajDB2(irow_new:irow_new+4,:) = trajDB(irow:irow+4,:);
end
evalDB2 = evalDB(idx_trj,:);

end

function [u,trajDB,evalDB]=DWA_with_VelocityObstacle(mycar,x,pedestrians,model,goal,evalParam,ob,R,dt)
%DWAによる入力値の計算をする関数

%Dynamic Window[vmin,vmax,ωmin,ωmax]の作成
Vr=CalcDynamicWindow(x,model,dt);
%評価関数の計算
[evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam,dt);

if isempty(evalDB)
    disp('no path to goal!!');
    u=[0;0];return;
end

%各評価関数の正規化
evalDB=NormalizeEval(evalDB);

%最終評価値の計算
feval=[];
for id=1:length(evalDB(:,1))
    feval=[feval;evalParam(1:3)*evalDB(id,3:5)'];
end
evalDB=[evalDB feval];

%------------------------------------------------
% Velocity ObstacleによるTrajectoryの選別
velObst = get_closeVelocityObstacle(mycar, pedestrians);
if ~isempty(velObst)
    [trajDB,evalDB] = select_traj_outside_VelocityObstacle(velObst,trajDB,evalDB);
end

if isempty(evalDB)
   u = [0,0]';
   return
end
feval = evalDB(:,end);
%------------------------------------------------

[maxv,ind]=max(feval);%最も評価値が大きい入力値のインデックスを計算
u=evalDB(ind,1:2)';%評価値が高い入力値を返す
end

function [trajDB2,evalDB2] = select_traj_outside_VelocityObstacle(VO,trajDB,evalDB)
% select all traj outside VelocityObstacle

%--------------------------------

nVelObs  = length(VO);
%----- searching velocity intersecting with velocity obstacles
ntraj   = size(evalDB,1);
idx_trj = [];
for i = 1:ntraj
    irow = 3 + (i-1)*5;
    %--- first 1 sec data
    theta = trajDB(irow,  1:5);
    speed = trajDB(irow+1,1:5);
    velocity = convert_Velocity(speed,theta);
    
    flag = 0;
    for j = 1:nVelObs
        int_xy = InterX(velocity, VO{j});
        if ~isempty(int_xy) % when intersecting
            flag = 1;
        end
    end
    if flag==0
       idx_trj = [idx_trj, i];
    end
end
%---------------------------------------------------------------------

if isempty(idx_trj)
   trajDB2 = [];
   evalDB2 = [];
   return
end

if length(idx_trj)==ntraj
   trajDB2 = trajDB;
   evalDB2 = evalDB;
   return
end

itraj  = length(idx_trj)*5;
jtraj  = size(trajDB,2);
trajDB2 = zeros(itraj,jtraj);
for i = 1:length(idx_trj)
    idx  = idx_trj(i);
    irow     =  1 + (idx-1)*5;
    irow_new = 1+(i-1)*5;
    trajDB2(irow_new:irow_new+4,:) = trajDB(irow:irow+4,:);
end
evalDB2 = evalDB(idx_trj,:);

end

function velocity = convert_Velocity(speed,theta)
%

speed2  = [speed;speed];
cos_sin = [cos(theta);sin(theta)];

velocity = speed2.*cos_sin;
end



function box = get_searchBOX(pedestrians,idx,evalParam)
% get searchigBOX made by future position of pedestrians

dt = evalParam(4);
one = ones(1,5);

pos  = pedestrians.person{idx}.pos(1:2);
theta= pedestrians.person{idx}.pos(3);
R    = pedestrians.person{idx}.D/2 + 1250;        
path = pedestrians.person{idx}.vel(1)*dt*1.1;
tmpbox  = [0, path, path, 0, 0; -R, -R, R, R, -R];
c    = cos(theta*pi/180);
s    = sin(theta*pi/180);
rotation = [c -s; s c];
    
box = rotation*tmpbox;
box(1,:) = box(1,:)+ pos(1)*one;
box(2,:) = box(2,:)+ pos(2)*one;

end


function [evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam,dt)
%各パスに対して評価値を計算する関数
evalDB=[];
trajDB=[];

for vt=Vr(1):model(5):Vr(2)
    for ot=Vr(3):model(6):Vr(4)
        %軌跡の推定
        [xt,traj]=GenerateTrajectory(x,vt,ot,evalParam(4),model,dt);
        %各評価関数の計算
        heading=CalcHeadingEval(xt,goal);
        dist=CalcDistEval(xt,ob,R);
        vel=abs(vt);
        
        evalDB=[evalDB;[vt ot heading dist vel]];
        trajDB=[trajDB;traj];     
    end
end

end

function EvalDB=NormalizeEval(EvalDB)
%評価値を正規化する関数
if sum(EvalDB(:,3))~=0
    EvalDB(:,3)=EvalDB(:,3)/sum(EvalDB(:,3));
end
if sum(EvalDB(:,4))~=0
    EvalDB(:,4)=EvalDB(:,4)/sum(EvalDB(:,4));
end
if sum(EvalDB(:,5))~=0
    EvalDB(:,5)=EvalDB(:,5)/sum(EvalDB(:,5));
end

end

function [x,traj]=GenerateTrajectory(x,vt,ot,evaldt,model,dt)
%軌跡データを作成する関数

time=0;
u=[vt;ot];%入力値
traj=x;%軌跡データ
while time<=evaldt
    time=time+dt;%シミュレーション時間の更新
    x=f(x,u,dt);%運動モデルによる推移
    traj=[traj x];
end
end

function stopDist=CalcBreakingDist(vel,model,dt)
%現在の速度から力学モデルに従って制動距離を計算する関数

stopDist=0;
while vel>0
    stopDist=stopDist+vel*dt;%制動距離の計算
    vel=vel-model(3)*dt;%最高原則
end
end


function dist=CalcDistEval(x,ob,R)
%障害物との距離評価値を計算する関数

nobstacle = size(ob,1);
one = ones(nobstacle,1);
pos = x(1:2)';

disttmp = ob - pos(ones(nobstacle,1),:);
disttmp2 = sqrt(sum(disttmp.^2, 2)) - one*R;

dist = min(disttmp2);

end


%{
function dist=CalcDistEval(x,ob,R)
%障害物との距離評価値を計算する関数

dist=1*10^10;
for io=1:length(ob(:,1))
    disttmp=norm(ob(io,:)-x(1:2)')-R;%パスの位置と障害物とのノルム誤差を計算
    if dist>disttmp%最小値を見つける
        dist=disttmp;
    end
end

end
%}

function heading=CalcHeadingEval(x,goal)
%headingの評価関数を計算する関数

theta=toDegree(x(3));%ロボットの方位
goalTheta=toDegree(atan2(goal(2)-x(2),goal(1)-x(1)));%ゴールの方位

if goalTheta>theta
    targetTheta=goalTheta-theta;%ゴールまでの方位差分[deg]
else
    targetTheta=theta-goalTheta;%ゴールまでの方位差分[deg]
end

heading=180-targetTheta;
end

function Vr=CalcDynamicWindow(x,model,dt)
%モデルと現在の状態からDyamicWindowを計算

%車両モデルによるWindow
Vs=[0 model(1) -model(2) model(2)];

%運動モデルによるWindow
Vd=[x(4)-model(3)*dt x(4)+model(3)*dt x(5)-model(4)*dt x(5)+model(4)*dt];

%最終的なDynamic Windowの計算
Vtmp=[Vs;Vd];
Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];
%[vmin,vmax,ωmin,ωmax]
end

function x = f(x, u,dt)
% Motion Model
 
F = [1 0 0 0 0
     0 1 0 0 0
     0 0 1 0 0
     0 0 0 0 0
     0 0 0 0 0];
 
B = [dt*cos(x(3)) 0
    dt*sin(x(3)) 0
    0 dt
    1 0
    0 1];

x= F*x+B*u;
end

function radian = toRadian(degree)
% degree to radian
radian = degree/180*pi;
end

function degree = toDegree(radian)
% radian to degree
degree = radian/pi*180;
end
