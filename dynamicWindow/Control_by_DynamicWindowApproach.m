function [mycar,traj,eval] = Control_by_DynamicWindowApproach(mycar, pedestrians, sim, goal,obstacle,obstacleR, Kinematic, evalParam, area, PLOT_DWA, FLAG_methodDWA)
% DynamicWindowApproach
%global dt;
persistent fist_flag h stop_flag

dt = sim.T;
if isempty(fist_flag)
    fist_flag = true;
end

pos = mycar.pos;
vel = mycar.vel;
x = [pos(1), pos(2), toRadian(pos(3)), vel(1), toRadian(vel(2))]';

if  FLAG_methodDWA==1
    [u,traj,eval]=DynamicWindowApproach(x,Kinematic,goal,evalParam,obstacle,obstacleR,dt);    
elseif FLAG_methodDWA==2    
    [u,traj,eval]=TimeVaryingDynamicWindowApproach(x,pedestrians, Kinematic,goal,evalParam,obstacle,obstacleR,dt);
elseif FLAG_methodDWA==3
    [u,traj,eval]=DWA_with_VelocityObstacle(mycar, x,pedestrians, Kinematic,goal,evalParam,obstacle,obstacleR,dt);
end

new_vel = [u(1), toDegree(u(2))];
%{
if mycar.pos(3) > 45
  new_vel =[-600 -1];
elseif mycar.pos(3) < -45
  new_vel =[-600 1];
end
%}

mycar.vel = new_vel;


if PLOT_DWA
    [trajX, trajY] = get_trajData(traj);
    if sim.tick > 1
    if fist_flag == true
       fist_flag = false;
       h = plot(trajX,trajY,'-m');
    elseif fist_flag == false
       h.XData = trajX;
       h.YData = trajY;
    end
    end
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
