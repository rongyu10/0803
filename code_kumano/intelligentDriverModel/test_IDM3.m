function  [traffic,forwardCarID] = test_IDM3(othercars,mycar)

 [traffic.order, traffic.mycar_idx] = get_allcars_info(othercars,mycar);
forwardCarID = get_forwardCarID(traffic.order, traffic.mycar_idx, othercars, mycar);
end

function [xorder, mycar_idx] = get_allcars_info(othercars,mycar)
% xorder: ID number in x-coord descending order

nr_cars = othercars.n;

allpos = get_allPosition(othercars, mycar);
xcoord = allpos(:,1);
id = 1:nr_cars+1;

[xtmp,itmp] = sort(xcoord,'descend');
xorder = id(itmp);
mycar_idx = find(xtmp==mycar.pos(1));

end

function forwardCarID = get_forwardCarID(xorder, mycar_idx,othercars, mycar)

ncars = length(xorder);
forwardCarID = zeros(1,ncars);

%allpos = get_allPosition(othercars, mycar);
allBD = get_allBoundary(othercars, mycar);

for i = 1:ncars
    if i == mycar_idx
        pos = mycar.pos;
    else
        pos = othercars.car{xorder(i)}.pos;
    end
    
    if i ==1
        forwardCarID(i) = 0;
    elseif pos(1)< 0 % outside of the simulater (X cood is negative)
        forwardCarID(i) = 0;
    else
        
       %searchbd = searchArea_ArcType(pos);
       searchbd = searchArea_RectType(pos);
                  
       iforward = get_forwardCarIndex(i, xorder, allBD, searchbd);
       %iforward = get_forwardCarIndex_by_InterX(i, xorder, allBD, searchbd);

       if iforward==0
          forwardCarID(i) = 0;
       else
          forwardCarID(i) = xorder(iforward);
       end
    end
end


end

function iforward = get_forwardCarIndex(i_mynum, xorder, allBD, searchbd)


flag    = zeros(i_mynum-1, 1);
for i = 1:i_mynum-1
    tmp_otherBD = allBD{xorder(i),1};
    otherBD = tmp_otherBD';
    %----    
    P = inpolygon(otherBD(:,1), otherBD(:,2), searchbd(:, 1), searchbd(:, 2));
    if sum(P)==0
        flag(i) = 0;
    else
        flag(i) = 1;
    end
    %----
end

tmp = sum(flag);
if tmp==0
   iforward = 0;
   return
else
   
   iforward = find(flag,1,'last');
   return
end


end


function allpos = get_allPosition(othercars, mycar)

nr_cars = othercars.n;
allpos  = zeros(nr_cars+1,2);

for i=1:nr_cars
    allpos(i,1:2) = othercars.car{i}.pos(1:2);
end
%--- my car position------
allpos(nr_cars+1,1:2) = mycar.pos(1:2);
%--------------------------

end

function bd = searchArea_ArcType(pos)
%-- Area Setting----
ANGLE  = 16*(pi/180); % 16 degree
%ANGLE  = 20*(pi/180); % 16 degree
RADIUS = 50*10^3;     % 30 m
%--------------------
theta  = linspace(-0.5*ANGLE, 0.5*ANGLE, 6)';

arc_x = RADIUS*cos(theta);
arc_y = RADIUS*sin(theta);

tmp_bd  = [0 0; arc_x arc_y; 0 0];

%--rotation-----
currdeg = pos(3);
c = cos(currdeg*pi/180);
s = sin(currdeg*pi/180);
rotaion = [c -s;s c]';
tmp_bd = tmp_bd*rotaion;
%----------------
nsize = size(tmp_bd,1);
bd = tmp_bd + repmat(pos(1:2),nsize,1);

end

function bd = searchArea_RectType(pos)
%-- Area Setting----
Length = 50*10^3;     % 30 m
H      = 2500;
W      = 4900;
%--------------------

tmp_bd  = [0 -H/2; Length -H/2; Length  H/2; 0  H/2; 0 -H/2];

%--rotation-----
currdeg = pos(3);
c = cos(currdeg*pi/180);
s = sin(currdeg*pi/180);
rotaion = [c -s;s c]';
tmp_bd = tmp_bd*rotaion;
%----------------
nsize = size(tmp_bd,1);
bd = tmp_bd + repmat(pos(1:2),nsize,1);

end


function allBD = get_allBoundary(othercars, mycar)

nr_cars = othercars.n;
allBD  = cell(nr_cars+1,1);

for i=1:nr_cars
   allBD{i,1} = get_carLineData(othercars.car{i}.bd);
end
%--- my car position------
 allBD{nr_cars+1,1} = get_carLineData(mycar.bd);
%--------------------------

end


function iforward = get_forwardCarIndex_by_InterX(i_mynum, xorder, allBD, searchbd)


flag    = zeros(i_mynum-1, 1);
for i = 1:i_mynum-1
    otherBD = allBD{xorder(i),1};
    %----    
    P = InterX(searchbd', otherBD);
    if isempty(P)
        flag(i) = 0;
    else
        flag(i) = 1;
    end
    %----
end

tmp = sum(flag);
if tmp==0
   iforward = 0;
   return
else
   iforward = find(flag,1,'last');
   return
end

end


function lineData = get_carLineData(car)

lineData = [car(1,1),car(2,1),car(4,1),car(5,1),car(7,1),car(8,1);... 
            car(1,2),car(2,2),car(4,2),car(5,2),car(7,2),car(8,2)];

end
