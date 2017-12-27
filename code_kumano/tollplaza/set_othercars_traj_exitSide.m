function othercars = set_othercars_traj_exitSide(othercars, traj_othercars, varargin)
% Setting trajectories data for Toll Plaza simulation

%-- open gate setting ----------
if nargin==2
    N_LANE  = 3;  % all gates are open
elseif nargin==3
    idx_openLane = varargin{1};
    N_LANE  = length(idx_openLane);
end
%-------------------------------

%--- make route data based on GATE number -----------
for icar = 1:othercars.n
   if nargin==2
      idx_exit = my_randsample(N_LANE, 1);
   else
      idx_tmp  = my_randsample(N_LANE, 1);
      idx_exit = idx_openLane(idx_tmp); 
   end
   idx_lane = othercars.car{icar}.save.lane_idx;
   idx_route = get_route_idx(idx_lane, idx_exit);
  
   route = [];
   for j =1:length(idx_route)
      if j==length(idx_route)
        tmp = traj_othercars.route{idx_route(j),1};
        tmp1= tmp(2:end,:);
      else       
        tmp = traj_othercars.route{idx_route(j),1};
        tmp1= tmp(1:end-1,:);
      end
      route = [route;tmp1];
   end
   
   othercars.traj{icar,1}.route = route;
   [total_len,seg_len,seg_ang] = arclength(route(:,1),route(:,2));
   othercars.traj{icar,1}.total_len = total_len;
   othercars.traj{icar,1}.seg_len   = seg_len;
   othercars.traj{icar,1}.seg_ang   = seg_ang;
   othercars.traj{icar,1}.idx_exit  = idx_exit; 
   othercars.traj{icar,1}.idx_route = idx_route;   
end
%-----------------------------------------------------

end

%%%  要修整！！！
function idx_route = get_route_idx(idx_lane, idx_exit)

if idx_exit == 1
    switch idx_lane
    case 1
        idx_route = [1;16;25];
    case 2
        idx_route = [2;16;25];
    case 3
        idx_route = [3;16;25];
    case 4
        idx_route = [4;16;25];
    case 5
        idx_route = [5;16;25];
    case 6
        idx_route = [6;19;25];
    case 7
        idx_route = [7;19;25];
    case 8
        idx_route = [8;19;25];
    case 9
        idx_route = [9;19;25];
    case 10
        idx_route = [10;19;25];
    case 11
        idx_route = [11;22;25];
    case 12
        idx_route = [12;22;25];
    case 13
        idx_route = [13;22;25];
    case 14
        idx_route = [14;22;25];
    case 15
        idx_route = [15;22;25];
    otherwise
        fprintf(2, 'WRONG GATE Index in set_othercars_traj \n');
    end
elseif idx_exit == 2
    switch idx_lane
    case 1
        idx_route = [1;17;26];
    case 2
        idx_route = [2;17;26];
    case 3
        idx_route = [3;17;26];
    case 4
        idx_route = [4;17;26];
    case 5
        idx_route = [5;17;26];
    case 6
        idx_route = [6;20;26];
    case 7
        idx_route = [7;20;26];
    case 8
        idx_route = [8;20;26];
    case 9
        idx_route = [9;20;26];
    case 10
        idx_route = [10;20;26];
    case 11
        idx_route = [11;23;26];
    case 12
        idx_route = [12;23;26];
    case 13
        idx_route = [13;23;26];
    case 14
        idx_route = [14;23;26];
    case 15
        idx_route = [15;23;26];
    otherwise
        fprintf(2, 'WRONG GATE Index in set_othercars_traj \n');
    end
elseif  idx_exit == 3
    switch idx_lane
    case 1
        idx_route = [1;18;27];
    case 2
        idx_route = [2;18;27];
    case 3
        idx_route = [3;18;27];
    case 4
        idx_route = [4;18;27];
    case 5
        idx_route = [5;18;27];
    case 6
        idx_route = [6;21;27];
    case 7
        idx_route = [7;21;27];
    case 8
        idx_route = [8;21;27];
    case 9
        idx_route = [9;21;27];
    case 10
        idx_route = [10;21;27];
    case 11
        idx_route = [11;24;27];
    case 12
        idx_route = [12;24;27];
    case 13
        idx_route = [13;24;27];
    case 14
        idx_route = [14;24;27];
    case 15
        idx_route = [28];
    otherwise
        fprintf(2, 'WRONG GATE Index in set_othercars_traj \n');
    end
else 
  fprintf(2, 'WRONG Lane Index in set_othercars_traj \n');
end

end


