function flag = is_carcrashed3(mycar,othercars,varargin)
%
%  USAGE
%--- Crash by othercars ----
%   if is_carcrashed3(mycar,othercars)    
%       fprintf(2, 'COLLISION OCCURRED. \n');
%       mycar.vel = [0 0];
%       mycar = init_mycar(get_posintrack(track, 2, 0, 1, 0),ini_vel);
%       myinfo= get_trackinfo(track, mycar.pos, othercars);
%       FLAG_LANECHANGE = 0;
%   end
%-----------------------------
%--- Crash by othercars and pedestrians ----
%   if is_carcrashed3(mycar,othercars,pedestrians)
%       fprintf(2, 'COLLISION OCCURRED. \n');
%       mycar.vel = [0 0];
%       mycar = init_mycar(get_posintrack(track, 1, 0, 1, 0),ini_vel);
%       pedestrians = reset_pedestrians(pedestrians, track);
%       othercars = reset_othercars2(othercars,track);
%   end
%-------------------------------------------

if nargin==2
    %---- Crash by othercars -----
    idx_nearCar = get_nearCar(mycar,othercars);
    if isempty(idx_nearCar)
       flag = 0;
       return
    else    
       othercarBD = get_othercarBD(othercars,idx_nearCar);
       obstableBD = othercarBD;
    end
    
elseif nargin==3
    %---- Crash by othercars and pedestrians -----
    pedestrians =varargin{1};
    idx_nearPerson = get_nearPerson(mycar,pedestrians);
    idx_nearCar    = get_nearCar(mycar,othercars);
    
    if isempty(idx_nearCar)&&isempty(idx_nearPerson)
       flag = 0;
       return
    elseif isempty(idx_nearCar)&&(~isempty(idx_nearPerson))
        pedestrianBD = get_pedestrianBD(pedestrians,idx_nearPerson);
        obstableBD   = pedestrianBD;
    elseif (~isempty(idx_nearCar))&&(isempty(idx_nearPerson))
        othercarBD = get_othercarBD(othercars,idx_nearCar);
        obstableBD   = othercarBD;
    else
        pedestrianBD = get_pedestrianBD(pedestrians,idx_nearPerson);
        othercarBD = get_othercarBD(othercars,idx_nearCar);
        obstableBD   = [pedestrianBD,othercarBD];
    end
end

mycarBD    = get_carLineData(mycar.bd);
P = InterX(mycarBD, obstableBD);

%----
if isempty(P)
    flag = 0;
    return
else
    flag = 1;
    return
end
%----
   

end


function idx_nearCar = get_nearCar(mycar,othercars)

DISTANCE = 20*10^3;     % 20m

mycar_pos = mycar.pos(1:2);
nr_cars = othercars.n;

idx_nearCar =[];
for i=1:nr_cars
    pos = othercars.car{i}.pos(1:2);
    diff= pos - mycar_pos;
    if norm(diff) < DISTANCE
      idx_nearCar= [idx_nearCar;i];
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

function othercarBD = get_othercarBD(othercars,idx_nearCar)

nr_cars = length(idx_nearCar);

othercarBD = zeros(2, 1E3);
nr_bd      = 0;
nr_curr_bd = 7;

for i = 1:nr_cars
    lineData = get_carLineData(othercars.car{idx_nearCar(i)}.bd);
    tmpBD = [lineData [NaN ; NaN]];
    othercarBD(:,nr_bd+1: nr_bd+nr_curr_bd) = tmpBD;
    nr_bd = nr_bd + nr_curr_bd;
end

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
