function flag = is_goal(mycar,myinfo,track,lane_goal)

if (mycar.pos(1) > track.xmax-4000)&&(myinfo.lane_idx==lane_goal)
    flag = true;
else
    flag = false;    
end

end
