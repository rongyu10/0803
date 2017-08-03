function closeVO = get_closeVelocityObstacle_othercars(mycar, othercars)
% get all Velocity Obstacles within the specific distance

DIST_JUDGE = 30*10^3;

pos = mycar.pos(1:2);
idx = 0;
closeVO =[];
for i = 1:othercars.n
   [vo, ~] = get_velocityObstacle_othercars(mycar, othercars.car{i});

   pos_othercar = othercars.car{i}.pos(1:2);
   if norm(pos_othercar,2) < DIST_JUDGE
      idx = idx +1;
      closeVO{idx} = vo;
   end
end

return
end