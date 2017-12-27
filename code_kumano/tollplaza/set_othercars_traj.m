function othercars = set_othercars_traj(othercars, traj_othercars, varargin)
% Setting trajectories data for Toll Plaza simulation

%-- open gate setting ----------
if nargin==2
    N_GATE  = 15;  % all gates are open
elseif nargin==3
    idx_openGate = varargin{1};
    N_GATE  = length(idx_openGate);
end
%-------------------------------

%--- make route data based on GATE number -----------
for icar = 1:othercars.n
   if nargin==2
      idx_gate = my_randsample(N_GATE, 1);
   else
      idx_tmp  = my_randsample(N_GATE, 1);
      idx_gate = idx_openGate(idx_tmp); 
   end
   idx_lane = othercars.car{icar}.save.lane_idx;
   idx_route = get_route_idx(idx_lane, idx_gate);
  
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
   othercars.traj{icar,1}.idx_gate  = idx_gate; 
   othercars.traj{icar,1}.idx_route = idx_route;   
end
%-----------------------------------------------------

end

function idx_route = get_route_idx(idx_lane, idx_gate)


if idx_lane == 1
    switch idx_gate
    case 1
        idx_route = [1;4;13];
    case 2
        idx_route = [1;4;14];
    case 3
        idx_route = [1;4;15];
    case 4
        idx_route = [1;4;16];
    case 5
        idx_route = [1;4;17];
    case 6
        idx_route = [1;7;18];
    case 7
        idx_route = [1;7;19];
    case 8
        idx_route = [1;7;20];
    case 9
        idx_route = [1;7;21];
    case 10
        idx_route = [1;7;22];
    case 11
        idx_route = [1;10;23];
    case 12
        idx_route = [1;10;24];
    case 13
        idx_route = [1;10;25];
    case 14
        idx_route = [1;10;26];
    case 15
        idx_route = [1;10;27];
    otherwise
        fprintf(2, 'WRONG GATE Index in set_othercars_traj \n');
    end
elseif idx_lane == 2
    switch idx_gate
    case 1
        idx_route = [2;5;13];
    case 2
        idx_route = [2;5;14];
    case 3
        idx_route = [2;5;15];
    case 4
        idx_route = [2;5;16];
    case 5
        idx_route = [2;5;17];
    case 6
        idx_route = [2;8;18];
    case 7
        idx_route = [2;8;19];
    case 8
        idx_route = [2;8;20];
    case 9
        idx_route = [2;8;21];
    case 10
        idx_route = [2;8;22];
    case 11
        idx_route = [2;11;23];
    case 12
        idx_route = [2;11;24];
    case 13
        idx_route = [2;11;25];
    case 14
        idx_route = [2;11;26];
    case 15
        idx_route = [2;11;27];
    otherwise
        fprintf(2, 'WRONG GATE Index in set_othercars_traj \n');
    end
elseif  idx_lane == 3
    switch idx_gate
    case 1
        idx_route = [3;6;13];
    case 2
        idx_route = [3;6;14];
    case 3
        idx_route = [3;6;15];
    case 4
        idx_route = [3;6;16];
    case 5
        idx_route = [3;6;17];
    case 6
        idx_route = [3;9;18];
    case 7
        idx_route = [3;9;19];
    case 8
        idx_route = [3;9;20];
    case 9
        idx_route = [3;9;21];
    case 10
        idx_route = [3;9;22];
    case 11
        idx_route = [3;12;23];
    case 12
        idx_route = [3;12;24];
    case 13
        idx_route = [3;12;25];
    case 14
        idx_route = [3;12;26];
    case 15
        idx_route = [3;28];
    otherwise
        fprintf(2, 'WRONG GATE Index in set_othercars_traj \n');
    end
else 
  fprintf(2, 'WRONG Lane Index in set_othercars_traj \n');
end

end


