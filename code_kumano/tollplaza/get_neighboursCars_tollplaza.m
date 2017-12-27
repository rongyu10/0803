function [idx_frontL,idx_frontR,idx_rearL,idx_rearR] = get_neighboursCars_tollplaza(mycar, othercars,varargin)

%--- Searching Radius -----
if (nargin == 3)
    MAX_RADIUS = varargin{1};
else
    MAX_RADIUS = 50*10^3;
end
%--------------------------

nr_cars     = othercars.n;
pos_mycar   = mycar.pos(1:2);
theta_mycar = mycar.pos(3);

pos_othercars = zeros(nr_cars,2);
for i=1:nr_cars
    pos_othercars(i,:) = othercars.car{i}.pos(1:2);
end

%--- get ID of othercars inside circle of radius MAX_RADIUS ----
dist1 = pos_othercars - pos_mycar(ones(nr_cars,1),:);
dist2 = sqrt(sum(dist1.^2, 2));

id_near = find(dist2 < MAX_RADIUS);
%---------------------------------------------------------------

if isempty(id_near)
    idx_frontL = 0;
    idx_frontR = 0;
    idx_rearL  = 0;
    idx_rearR  = 0;
    return
end

%--- checking wchich quadrant each car belongs to ----- 
nr_carNear = size(id_near,1);
id_1quadrant = [];
id_2quadrant = [];
id_3quadrant = [];
id_4quadrant = [];
THRESHOLD1 = 3*pi/180; 
THRESHOLD2 = 177*pi/180; 

for i=1:nr_carNear
    vec_ego_to_other = othercars.car{id_near(i)}.pos(1:2) - pos_mycar;
    vec_ego          = get_directionVector(pos_mycar, theta_mycar) - pos_mycar;
    
    innerProd = dot(vec_ego,vec_ego_to_other);
    vec_ego_to_other_m = [vec_ego_to_other 0];
    vec_ego_m          = [vec_ego 0];
    crossProd_tmp      = cross(vec_ego_m, vec_ego_to_other_m);
    crossProd          = crossProd_tmp(3);
    
    theta = acos(innerProd/norm(vec_ego)/norm(vec_ego_to_other_m));
    if (theta> THRESHOLD2)||(theta < THRESHOLD1) % remove exact front/rear cars (cars on the same lane)
        continue
    end
    
    if (crossProd >=0)&&(innerProd>=0)
       id_1quadrant = [id_1quadrant id_near(i)];
    elseif (crossProd >=0)&&(innerProd<0)
       id_2quadrant = [id_2quadrant id_near(i)];
    elseif (crossProd <0)&&(innerProd<0)
       id_3quadrant = [id_3quadrant id_near(i)];
    elseif (crossProd <0)&&(innerProd>=0)
       id_4quadrant = [id_4quadrant id_near(i)];       
    end
end
%---------------------------------------------------------

idsize1 = length(id_1quadrant);
idsize2 = length(id_2quadrant);
idsize3 = length(id_3quadrant);
idsize4 = length(id_4quadrant);

%---- 1st quadrant------------------
if idsize1 == 0
    idx_frontL = 0;
elseif idsize1 == 1
    idx_frontL = id_1quadrant(1);
else
    tmp_dist2   = dist2(id_1quadrant);
    [~,idx_min] = min(tmp_dist2); % get ID with minium distance
    idx_frontL  = id_1quadrant(idx_min);
end
%------------------------------------
%---- 2nd quadrant------------------
if idsize2 == 0
    idx_rearL = 0;
elseif idsize2 == 1
    idx_rearL = id_2quadrant(1);
else
    tmp_dist2   = dist2(id_2quadrant);
    [~,idx_min] = min(tmp_dist2); % get ID with minium distance
    idx_rearL   = id_2quadrant(idx_min);
end
%------------------------------------
%---- 3rd quadrant------------------
if idsize3 == 0
    idx_rearR = 0;
elseif idsize3 == 1
    idx_rearR = id_3quadrant(1);
else
    tmp_dist2   = dist2(id_3quadrant);
    [~,idx_min] = min(tmp_dist2); % get ID with minium distance
    idx_rearR   = id_3quadrant(idx_min);
end
%------------------------------------
%---- 4th quadrant------------------
if idsize4 == 0
    idx_frontR = 0;
elseif idsize4 == 1
    idx_frontR = id_4quadrant(1);
else
    tmp_dist2   = dist2(id_4quadrant);
    [~,idx_min] = min(tmp_dist2); % get ID with minium distance
    idx_frontR  = id_4quadrant(idx_min);
end
%------------------------------------

%-- for debug-------
%fprintf('idx: %d, %d, %d, %d \n', idx_frontL,idx_frontR,idx_rearL,idx_rearR);
%-------------------
return
end


function vec_direction = get_directionVector(pos, theta)

unitVec = [10*10^3 0];
c    = cos(theta*pi/180);
s    = sin(theta*pi/180);
rotation = [c -s; s c]';

vec_tmp = unitVec*rotation;
vec_direction =vec_tmp + pos;

end
