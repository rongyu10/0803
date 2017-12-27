function traj_othercars = get_othercars_trajectories_tollplaza_exitSide()

nPt       = 100;
nPt_mid   = 50;
nPt_small = 10;
%- Sec1 (part1)----------------------------------------------------
ORIGIN_X = 45*10^3;
ORIGIN_Y = 0.0;
%--line1----
tmp_ctlPt = [0, 72.5; 10, 72.5; 50, 65.0; 100, 55.0; 150, 30.0; 160, 27.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj1, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line2----
tmp_ctlPt = [0, 67.5; 10, 67.5; 50, 62.5; 100, 50.0; 150, 30.0; 160, 27.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj2, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line3----
tmp_ctlPt = [0, 62.5; 10, 62.5; 50, 60.0; 100, 45.0; 150, 30.0; 160, 27.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj3, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line4----
tmp_ctlPt = [0, 57.5; 10, 57.5; 50, 57.5; 100, 40.0; 150, 30.0; 160, 27.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj4, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line5----
tmp_ctlPt = [0, 52.5; 10, 52.5; 50, 52.5; 100, 37.0; 150, 30.0; 160, 27.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj5, ~] = bezierCurve(ctlPt,nPt);
%-----------
route{1,1} = traj1;
route{2,1} = traj2;
route{3,1} = traj3;
route{4,1} = traj4;
route{5,1} = traj5;
%--------------------------------------------------------------------------
%- Sec1 (part2)----------------------------------------------------
ORIGIN_X = 45*10^3;
ORIGIN_Y = 0.0;
%--line1----
tmp_ctlPt = [0, 47.5; 10, 47.5; 50, 47.5; 100, 42.5; 150, 17.5; 160, 15.0]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj1, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line2----
tmp_ctlPt = [0, 42.5; 10, 42.5; 50, 42.5; 100, 37.5; 150, 17.5; 160, 15.0]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj2, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line3----
tmp_ctlPt = [0, 37.5; 10, 37.5; 50, 37.5; 100, 32.5; 150, 17.5; 160, 15.0]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj3, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line4----
tmp_ctlPt = [0, 32.5; 10, 32.5; 50, 32.5; 100, 27.5; 150, 17.5; 160, 15.0]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj4, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line5----
tmp_ctlPt = [0, 27.5; 10, 27.5; 50, 27.5; 100, 22.5; 150, 17.5; 160, 15.0]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj5, ~] = bezierCurve(ctlPt,nPt);
%-----------
route{6,1} = traj1;
route{7,1} = traj2;
route{8,1} = traj3;
route{9,1} = traj4;
route{10,1}= traj5;
%--------------------------------------------------------------------------
%- Sec1 (part3)----------------------------------------------------
ORIGIN_X = 45*10^3;
ORIGIN_Y = 0.0;
%--line1----
tmp_ctlPt = [0, 22.5; 10, 22.5; 50, 25.0; 100, 22.5; 150, 6.0; 160, 6.0]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj1, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line2----
tmp_ctlPt = [0, 17.5; 10, 17.5; 50, 20.0; 100, 17.5; 150, 6.0; 160, 6.0]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj2, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line3----
tmp_ctlPt = [0, 12.5; 10, 12.5; 50, 15.0; 100, 12.5; 150, 6.0; 160, 6.0]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj3, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line4----
tmp_ctlPt = [0, 7.5; 10, 7.5; 50, 10.0; 100, 7.5; 150, 6.0; 160, 6.0]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj4, ~] = bezierCurve(ctlPt,nPt);
%-----------
%--line5----
tmp_ctlPt = [0, 2.5; 10, 2.5; 50, 2.5; 100, 5.0; 150, 6.0; 160, 6.0]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj5, ~] = bezierCurve(ctlPt,nPt);
%-----------
route{11,1} = traj1;
route{12,1} = traj2;
route{13,1} = traj3;
route{14,1} = traj4;
route{15,1} = traj5;
%--------------------------------------------------------------------------
%- Sec2 (part1)----------------------------------------------------
ORIGIN_X = 205*10^3;
ORIGIN_Y = 0.0;
%--line1----
tmp_ctlPt = [0, 27.5; 10, 25.0; 35, 15.0; 60, 8.75; 70, 8.75]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj1, ~] = bezierCurve(ctlPt,nPt_mid);
%-----------
%--line2----
tmp_ctlPt = [0, 27.5; 10, 25.0; 35, 10; 60, 5.25; 70, 5.25]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj2, ~] = bezierCurve(ctlPt,nPt_mid);
%-----------
%--line3----
tmp_ctlPt = [0, 27.5; 10, 25.0; 35, 5; 60, 1.75; 70, 1.75]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj3, ~] = bezierCurve(ctlPt,nPt_mid);
%-----------
%}
route{16,1} = traj1;
route{17,1} = traj2;
route{18,1} = traj3;
%--------------------------------------------------------------------------
%- Sec2 (part2)----------------------------------------------------
ORIGIN_X = 205*10^3;
ORIGIN_Y = 0.0;
%--line1----
tmp_ctlPt = [0, 15.0; 10, 12.5; 35, 10.5; 60, 8.75; 70, 8.75]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj1, ~] = bezierCurve(ctlPt,nPt_mid);
%-----------
%--line2----
tmp_ctlPt = [0, 15.0; 10, 12.5; 35, 8.5; 60, 5.25; 70, 5.25]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj2, ~] = bezierCurve(ctlPt,nPt_mid);
%-----------
%--line3----
tmp_ctlPt = [0, 15.0; 10, 12.5; 35, 6.5; 60, 1.75; 70, 1.75]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj3, ~] = bezierCurve(ctlPt,nPt_mid);
%-----------
%}
route{19,1} = traj1;
route{20,1} = traj2;
route{21,1} = traj3;
%--------------------------------------------------------------------------
%- Sec2 (part3)----------------------------------------------------
ORIGIN_X = 205*10^3;
ORIGIN_Y = 0.0;
%--line1----
tmp_ctlPt = [0, 6.0; 10, 6.0; 35, 7.5; 60, 8.75; 70, 8.75]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj1, ~] = bezierCurve(ctlPt,nPt_mid);
%-----------
%--line2----
tmp_ctlPt = [0, 6.0; 10, 6.0; 35, 5.5; 60, 5.25; 70, 5.25]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj2, ~] = bezierCurve(ctlPt,nPt_mid);
%-----------
%--line3----
tmp_ctlPt = [0, 6.0; 10, 6.0; 35, 3.5; 60, 1.75; 70, 1.75]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj3, ~] = bezierCurve(ctlPt,nPt_mid);
%-----------
route{22,1} = traj1;
route{23,1} = traj2;
route{24,1} = traj3;
%--------------------------------------------------------------------------
%- Sec3 ----------------------------
ORIGIN_X = 275*10^3;
ORIGIN_Y = 0.0*10^3;
dx = 10;
dy = 3.5;
%--line1----
tmp_ctlPt = [0, dy*2.5; dx, dy*2.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj1, ~] = bezierCurve(ctlPt,nPt_small);
%-----------
%--line2----
tmp_ctlPt = [0, dy*1.5; dx, dy*1.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj2, ~] = bezierCurve(ctlPt,nPt_small);
%-----------
%--line3----
tmp_ctlPt = [0, dy*0.5; dx, dy*0.5]*10^3;
ctlPt = translatePoint(tmp_ctlPt, ORIGIN_X, ORIGIN_Y);
[traj3, ~] = bezierCurve(ctlPt,nPt_small);
%-----------
route{25,1} = traj1;
route{26,1} = traj2;
route{27,1} = traj3;
%-------------------------------------

%--STRAIGHT--------------------------------------------------------------------------
ORIGIN_X = 45*10^3;
ORIGIN_Y = 0.0;
dx = 60.0;
dy = 5.0;
tmp_ctlPt = [0, dy*0.5; dx, dy*0.5; dx*2, dy*0.5; dx*3, 1.75; dx*4, 1.75;]*10^3;
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