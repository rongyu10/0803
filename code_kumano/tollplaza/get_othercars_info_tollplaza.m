function othercars = get_othercars_info_tollplaza(road,othercars,varargin)

nr_cars = othercars.n;

%---- Enter-side/ Exixt-side of Toll Plaza -------
if (nargin == 2)
   enterFlag = true;
elseif (nargin == 3)&&(strcmp(varargin{1},'enter'))
   enterFlag = true;     
elseif (nargin == 3)&&(strcmp(varargin{1},'exit'))
   enterFlag = false; 
end
%--------------------------------------------------

for i = 1:nr_cars
    pos = othercars.car{i, 1}.pos;
    
    if enterFlag  % Enter-Side
        if (pos(1) > 100*10^3)&&(pos(1) < 275*10^3)
            othercars.info{i, 1}.type = 'plaza';
        else
            %othercars.info{i,1} = get_trackinfo_tollplaza(road, pos);
            othercars.info{i, 1}.type = 'straight';
        end
    else          % Eixt-Side
        if (pos(1) > 45*10^3)&&(pos(1) < 285*10^3)
            othercars.info{i, 1}.type = 'plaza';
        else
            %othercars.info{i,1} = get_trackinfo_tollplaza(road, pos);
            othercars.info{i, 1}.type = 'straight';
        end        
    end
    
end


end


