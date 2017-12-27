function traj_othercars = get_othercars_trajectories_60m_tollplaza()

nPt = 100;
%- Sec1 to Sec2---------------------
ORIGIN_X = 100*10^3;
ORIGIN_Y = 0.0*10^3;
dx = 15;
dy = 3.5;
%--line1----
tmp_ctlPt = [0, dy*2.5; dx, dy*2.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj1, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line2----
tmp_ctlPt = [0, dy*1.5; dx, dy*1.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj2, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line3----
tmp_ctlPt = [0, dy*0.5; dx, dy*0.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj3, ~] = bezierCurve(ctlPt,nPt);
%-----------
route{1,1} = traj1;
route{2,1} = traj2;
route{3,1} = traj3;
%-------------------------------------

%- Sec2 to Sec3 (part1)---------------------
ORIGIN_X = 115*10^3;
ORIGIN_Y = 0.0;
dx = 15.0;
dy = 3.5;
%--line1----
tmp_ctlPt = [0, dy*2.5; dx, dy*2.5; 30, 20; 50, 30; 60, 35]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj1, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line2----
tmp_ctlPt = [0, dy*1.5; dx, dy*1.5; 30, 15; 50, 30; 60, 35]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj2, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line3----
tmp_ctlPt = [0, dy*0.5; dx, dy*0.5; 30, 10; 50, 30; 60, 35]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj3, ~] = bezierCurve(ctlPt,nPt);
%-----------
route{4,1} = traj1;
route{5,1} = traj2;
route{6,1} = traj3;
%-------------------------------------
%- Sec2 to Sec3 (part2)---------------------
ORIGIN_X = 115*10^3;
ORIGIN_Y = 0.0;
dx = 15.0;
dy = 3.5;
%--line1----
tmp_ctlPt = [0, dy*2.5; dx, dy*2.5; 30, 15; 50, 18.5; 60, 21.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj1, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line2----
tmp_ctlPt = [0, dy*1.5; dx, dy*1.5; 30, 10; 50, 18.5; 60, 21.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj2, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line3----
tmp_ctlPt = [0, dy*0.5; dx, dy*0.5; 30, 5;  50, 18.5; 60, 21.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj3, ~] = bezierCurve(ctlPt,nPt);
%-----------
route{7,1} = traj1;
route{8,1} = traj2;
route{9,1} = traj3;
%-------------------------------------

%- Sec2 to Sec3 (part3)---------------------
ORIGIN_X = 115*10^3;
ORIGIN_Y = 0.0;
dx = 15.0;
dy = 3.5;
%--line1----
tmp_ctlPt = [0, dy*2.5; dx, dy*2.5; 30, 10; 50, 7; 60, 7]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj1, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line2----
tmp_ctlPt = [0, dy*1.5; dx, dy*1.5; 30, 6;  50, 7; 60, 7]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj2, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line3----
tmp_ctlPt = [0, dy*0.5; dx, dy*0.5; 30, 2.5; 50, 7; 60, 7]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj3, ~] = bezierCurve(ctlPt,nPt);
%-----------
route{10,1} = traj1;
route{11,1} = traj2;
route{12,1} = traj3;
%-------------------------------------

%- Sec3 to Sec4 (part1)----------------------------------------------------
ORIGIN_X = 115*10^3;
ORIGIN_Y = 0.0;
%--line1----
tmp_ctlPt = [60, 35.0; 70, 40.0; 110, 65.0; 130, 70.0; 150, 72.5; 160, 72.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj1, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line2----
tmp_ctlPt = [60, 35.0; 70, 40.0; 110, 60.0; 130, 65.0; 150, 67.5; 160, 67.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj2, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line3----
tmp_ctlPt = [60, 35.0; 70, 40.0; 110, 55.0; 130, 60.0; 150, 62.5; 160, 62.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj3, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line4----
tmp_ctlPt = [60, 35.0; 70, 40.0; 110, 50.0; 130, 55.0; 150, 57.5; 160, 57.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj4, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line5----
tmp_ctlPt = [60, 35.0; 70, 40.0; 110, 45.0; 130, 51.0; 150, 52.5; 160, 52.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj5, ~] = bezierCurve(ctlPt,nPt);
%-----------
route{13,1} = traj1;
route{14,1} = traj2;
route{15,1} = traj3;
route{16,1} = traj4;
route{17,1} = traj5;
%--------------------------------------------------------------------------

%- Sec3 to Sec4 (part2)----------------------------------------------------
ORIGIN_X = 115*10^3;
ORIGIN_Y = 0.0;
%--line1----
tmp_ctlPt = [60, 21.5; 70, 24.0; 110, 45.0; 130, 45.0; 150, 47.5; 160, 47.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj1, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line2----
tmp_ctlPt = [60, 21.5; 70, 24.0; 110, 40.0; 130, 40.0; 150, 42.5; 160, 42.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj2, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line3----
tmp_ctlPt = [60, 21.5; 70, 24.0; 110, 35.0; 130, 35.0; 150, 37.5; 160, 37.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj3, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line4----
tmp_ctlPt = [60, 21.5; 70, 24.0; 110, 30.0; 130, 30.0; 150, 32.5; 160, 32.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj4, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line5----
tmp_ctlPt = [60, 21.5; 70, 24.0; 110, 25.0; 130, 25.0; 150, 27.5; 160, 27.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj5, ~] = bezierCurve(ctlPt,nPt);
%-----------
route{18,1} = traj1;
route{19,1} = traj2;
route{20,1} = traj3;
route{21,1} = traj4;
route{22,1} = traj5;
%--------------------------------------------------------------------------

%- Sec3 to Sec4 (part3)----------------------------------------------------
ORIGIN_X = 115*10^3;
ORIGIN_Y = 0.0;
%--line1----
tmp_ctlPt = [60, 7; 70,  7; 110, 20.0; 130, 22.5; 150, 22.5; 160, 22.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj1, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line2----
tmp_ctlPt = [60, 7; 70,  7; 110, 15.0; 130, 17.5; 150, 17.5; 160, 17.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj2, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line3----
tmp_ctlPt = [60, 7; 70,  7; 110, 10.0; 130, 12.5; 150, 12.5; 160, 12.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj3, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line4----
tmp_ctlPt = [60, 7; 70,  7; 110, 5.0; 130, 7.5; 150, 7.5; 160, 7.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj4, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line5----
tmp_ctlPt = [60, 7; 70,  7; 110, 1.0; 130, 2.5; 150, 2.5; 160, 2.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj5, ~] = bezierCurve(ctlPt,nPt);
%-----------
route{23,1} = traj1;
route{24,1} = traj2;
route{25,1} = traj3;
route{26,1} = traj4;
route{27,1} = traj5;
%--------------------------------------------------------------------------

%--STRAIGHT--------------------------------------------------------------------------
ORIGIN_X = 115*10^3;
ORIGIN_Y = 0.0;
dx = 15.0;
dy = 3.5;
tmp_ctlPt = [0, dy*0.5; dx, dy*0.5; 30, 2.5; 50, 2.5; 80, 2.5; 110, 2.5; 130, 2.5; 150, 2.5; 160, 2.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj1, ~] = bezierCurve(ctlPt,nPt);
route{end+1,1} = traj1;
%-------------------------------------------------------------------------------------


traj_othercars.route = route;
traj_othercars.n     = size(route,1);

[xmin, xmax, ymin, ymax] = get_xyminmax(route);
traj_othercars.xmin = xmin;
traj_othercars.xmax = xmax;
traj_othercars.ymin = ymin;
traj_othercars.ymax = ymax;
end

function tranPt = translatePoint(Pt,dX, dY)

isize = size(Pt,1);
jsize = size(Pt,2);
tranPt = zeros(isize,jsize);

for i = 1:isize
    tranPt(i,1) = Pt(i,1) + dX;
    tranPt(i,2) = Pt(i,2) + dY;    
end
end

function [xmin, xmax, ymin, ymax] = get_xyminmax(route)
%
%

n_traj     = size(route,1);

xmin = zeros(n_traj,1);
xmax = zeros(n_traj,1);
ymin = zeros(n_traj,1);
ymax = zeros(n_traj,1);

for i = 1:n_traj
   tmp_data = route{i,1};
   xdata    = tmp_data(:,1);
   ydata    = tmp_data(:,2);
   xmin(i,1)= min(xdata);
   xmax(i,1)= max(xdata);
   ymin(i,1)= min(ydata);
   ymax(i,1)= max(ydata);
end

end