function [mycar,traj,eval] = Control_by_DynamicWindowApproach(mycar, traffic_participants, sim, goal,obstacle,obstacleR, Kinematic, evalParam, area, PLOT_DWA, FLAG_methodDWA, varargin)
% DynamicWindowApproach
%global dt;
persistent fist_flag h

dt = sim.T;
if isempty(fist_flag)
    fist_flag = true;
end

pos = mycar.pos;
vel = mycar.vel;
x = [pos(1), pos(2), toRadian(pos(3)), vel(1), toRadian(vel(2))]';


% u: next step action ( u(1): velocity, u(2): angular velocity)  
% traj: size(traj,2):  number of time steps (prediction time/time step size)
%       size(traj,1)/5: number of all path candidates
%       traj(i1,:): X-cood of trajectory
%       traj(i2,:): Y-cood of trajectory
%       traj(i3,:): Angle  of trajectory in radian
%       traj(i4,:): velocity 
%       traj(i5,:): angular velocity (rad/sec)
%
%------------------------------------------------------
% eval: evaluation table of all path candidates 
%       size(eval,1): number of all path candidates
%       eval(:,1)= u(1) velocity, eval(:,2)= u(2) angular velocity, 
%       eval(:,3)= obj1, eval(:,4)= obj2, eval(:,5)= obj3
%       eval(:,6)= utility function value (obj1*w1 + obj2*w2 obj3*w3)
%-------------------------------------------------------


if  FLAG_methodDWA==1
    [u,traj,eval]=DynamicWindowApproach(x,Kinematic,goal,evalParam,obstacle,obstacleR,dt);    
elseif FLAG_methodDWA==2
    pedestrians = traffic_participants;
    [u,traj,eval]=TimeVaryingDynamicWindowApproach(x,pedestrians, Kinematic,goal,evalParam,obstacle,obstacleR,dt);
elseif FLAG_methodDWA==3
    pedestrians = traffic_participants;
    [u,traj,eval]=DWA_with_VelocityObstacle(mycar, x,pedestrians, Kinematic,goal,evalParam,obstacle,obstacleR,dt);
elseif FLAG_methodDWA==4
    othercars = traffic_participants;
    [u,traj,eval]=DWA_with_VelocityObstacle_othercars(mycar, x, othercars, Kinematic,goal,evalParam,obstacle,obstacleR,dt);
elseif FLAG_methodDWA==5
    risk  = varargin{1};
    risk_rearL  = risk.rearL; 
    risk_rearR  = risk.rearR;
    risk_frontL = risk.frontL;
    risk_frontR = risk.frontR;
    [u,traj,eval]=DynamicWindowApproach_SteeringConstraints(x,Kinematic,goal,evalParam,obstacle,obstacleR,dt,risk_rearL,risk_rearR ); 
%{    
    if ((risk_rearL==1)||(risk_frontL==1))&&(u(2)>0)
        u(2)=0;
    elseif ((risk_rearR==1)||(risk_frontR==1))&&(u(2)<0)
        u(2)=0;        
    end
%}
    if (risk_frontL==1)&&(u(2)>0)
        %u(1)=u(1) -2*10^3*dt;
        u(2)=0;
    elseif (risk_frontR==1)&&(u(2)<0)
        %u(1)=u(1) -2*10^3*dt;
        u(2)=0;
    elseif (risk_rearL==1)&&(u(2)>0)
        u(2)=0;
    elseif (risk_rearR==1)&&(u(2)<0)
        u(2)=0;
    end    
end

%--- Update mycar velocity -----------
if isempty(eval)
    new_vel = mycar.vel; % for failed case
    %new_vel = [0 0];
else
    new_vel = [u(1), toDegree(u(2))]; % DWA result
end
mycar.vel = new_vel;
%-------------------------------------

%-- plot DWA trajectories -----------
if PLOT_DWA
    [trajX, trajY] = get_trajData(traj);
    %-- for debug case------------
    if (isempty(trajX))||(isempty(trajY))
        return
    end
    %------------------------------
    if sim.tick > 1
    if fist_flag == true
       fist_flag = false;
       h = plot(trajX,trajY,'-m');
    elseif fist_flag == false
       h.XData = trajX;
       h.YData = trajY;
    end
    end
else
    h.XData = [];
    h.YData = [];
    fist_flag =[]; 
end
%------------------------------------

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

if length(trajX)~=length(trajY)
    trajX = [];
    trajY = [];
end

end

function [u,trajDB,evalDB]=DynamicWindowApproach(x,model,goal,evalParam,ob,R,dt)
%DWAによる入力値の計算をする関数

%Dynamic Window[vmin,vmax,ωmin,ωmax]の作成
Vr=CalcDynamicWindow(x,model,dt);
%評価関数の計算
[evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam,dt);

if isempty(evalDB)
    %disp('no path to goal!!');
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

%-------------------------------------------------------------------
function [u,trajDB,evalDB]=DynamicWindowApproach_SteeringConstraints(x,model,goal,evalParam,ob,R,dt,risk_Left,risk_Right)
%DWAによる入力値の計算をする関数

%Dynamic Window[vmin,vmax,ωmin,ωmax]の作成
Vr=CalcDynamicWindow_SteeringConstraints(x,model,dt,risk_Left,risk_Right); 

%評価関数の計算
[evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam,dt);

if isempty(evalDB)
    %disp('no path to goal!!');
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
%-------------------------------------------------------------------

function [u,trajDB,evalDB]=TimeVaryingDynamicWindowApproach(x,pedestrians,model,goal,evalParam,ob,R,dt)
%DWAによる入力値の計算をする関数

%Dynamic Window[vmin,vmax,ωmin,ωmax]の作成
Vr=CalcDynamicWindow(x,model,dt);
%評価関数の計算
[evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam,dt);

if isempty(evalDB)
    %disp('no path to goal!!');
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

function [u,trajDB,evalDB]=DWA_with_VelocityObstacle_othercars(mycar,x,othercars,model,goal,evalParam,ob,R,dt)
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
% Velocity ObstacleによるTrajectoryの選別 (1sec後のVelocityでスクリーニング)
maxradius  = 40*10^3;
velObst = get_closeVelocityObstacle_othercars(mycar, othercars,maxradius);

if ~isempty(velObst)
    NSTEP_1sec = round(1.0/dt)+1;
    [trajDB,evalDB] = select_traj_outside_VelocityObstacle_othercars(velObst,trajDB,evalDB,NSTEP_1sec);
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

function [trajDB2,evalDB2] = select_traj_outside_VelocityObstacle_othercars(VO,trajDB,evalDB,NSTEP_1sec)
% select all traj outside VelocityObstacle

%--------------------------------

nVelObs  = length(VO);
%----- calc velocity 1sec later and checking if the velocity is inside the velocity obstacles
ntraj   = size(evalDB,1);
idx_trj = [];


for i = 1:ntraj
    irow =(i-1)*5+1;
    pos_0sec = trajDB(irow:irow+1,1);
    pos_1sec = trajDB(irow:irow+1,NSTEP_1sec);
    velocity = (pos_1sec - pos_0sec)/1.0;
    
    % ---  checking if the velocity is inside the velocity obstacles
    flag = 0;    
    for j = 1:nVelObs
        vo_j = VO{j};
        in = inpolygon(velocity(1),velocity(2), vo_j(:,1), vo_j(:,2));
        if (in == 1) % when inside the velocity obstacle
            flag = 1;
            break
        end
    end
    %---------------------------------------------------------------  
    if flag==0 % when outside the velocity obstacle
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
    irow_new =  1 + (i-1)*5;
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

jdata = ceil(evaldt/dt)+1;
traj  = zeros(5,jdata);
%traj=x;%軌跡データ
traj(:,1)=x;%軌跡データ

F = [1 0 0 0 0
     0 1 0 0 0
     0 0 1 0 0
     0 0 0 0 0
     0 0 0 0 0];

B =[1 0
    1 0
    0 dt
    1 0
    0 1];
j = 1;
while time<=evaldt
    time=time+dt;%シミュレーション時間の更新
    j = j+1;
    x=f(F,B,x,u,dt);%運動モデルによる推移
    %traj=[traj x];
    traj(:,j)= x;
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

function Vr=CalcDynamicWindow_SteeringConstraints(x,model,dt,risk_Left,risk_Right)
%モデルと現在の状態からDyamicWindowを計算

%車両モデルによるWindow
Vs=[0 model(1) -model(2) model(2)];

%運動モデルによるWindow
%--- SteeringConstraints ---

%{
if (risk_Left==1)&&(risk_Right==1)
  omega_left  = model(4)*0.1;
  omega_right = model(4)*0.1;
elseif (risk_Left==1)&&(risk_Right==0)
  omega_left = 0;
  omega_right = model(4);
elseif (risk_Left==0)&&(risk_Right==1)
  omega_left  = model(4);  
  omega_right = 0;
else
  omega_left  = model(4);  
  omega_right = model(4);    
end
%}


if risk_Left==1
  omega_left = 0;
else
  omega_left = model(4);
end
%----
if risk_Right==1
  omega_right = 0;
else
  omega_right = model(4);
end
%}
%---------------------------

Vd=[x(4)-model(3)*dt x(4)+model(3)*dt x(5)-model(4)*dt x(5)+model(4)*dt];
%Vd=[x(4)-model(3)*dt x(4)+model(3)*dt x(5)-omega_right*dt x(5)+omega_left*dt];

%最終的なDynamic Windowの計算
Vtmp=[Vs;Vd];
Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];
%[vmin,vmax,ωmin,ωmax]
end



function x = f(F,B, x, u,dt)
% Motion Model
 
% F = [1 0 0 0 0
%      0 1 0 0 0
%      0 0 1 0 0
%      0 0 0 0 0
%      0 0 0 0 0];

% B = [dt*cos(x(3)) 0
%     dt*sin(x(3)) 0
%     0 dt
%     1 0
%     0 1];

B(1,1) = dt*cos(x(3));
B(2,1) = dt*sin(x(3));

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
