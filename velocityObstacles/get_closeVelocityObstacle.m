function closeVO = get_closeVelocityObstacle(mycar, pedestrians)
% get all Velocity Obstacles within the specific distance

DIST_JUDGE = 15*10^3;

pos = mycar.pos(1:2);
idx = 0;
closeVO =[];
for i = 1:pedestrians.n
   [vo, ~] = get_velocityObstacle(mycar, pedestrians.person{i});

   pos_pedestrian = pedestrians.person{i}.pos(1:2);
   if norm(pos_pedestrian-pos,2) < DIST_JUDGE
      idx = idx +1;
      closeVO{idx} = vo;
   end
end

return
end