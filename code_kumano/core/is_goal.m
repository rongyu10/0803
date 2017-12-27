function flag = is_goal(mycar,myinfo,track,lane_goal,varargin)

%--- Goal Track and Lane----
if (nargin == 5)
   track_goal =  varargin{1};
   if (myinfo.track_idx == track_goal)&&(myinfo.lane_idx==lane_goal)
      flag = true;
      return
   else
      flag = false;
      return
  end      
end
%---------------------------

%--- Goal x coord and Lane---
if (mycar.pos(1) > track.xmax-4000)&&(myinfo.lane_idx==lane_goal)
    flag = true;
else
    flag = false;    
end
%-----------------------------
end
