function othercars = reset_othercars2(othercars,track)
% RESET OTHERCARS
xmax = track.xmax;
xmin = track.xmin +10000;

for i = 1:othercars.n
    xpos = (xmax - xmin)*rand() + xmin;
    othercars.car{i}.pos(1) = xpos;
end

%------------------------------
if abs(othercars.car{1}.pos(1)-othercars.car{2}.pos(1)) < 10000
  flag = true;
  while(flag)
    xpos = (xmax - xmin)*rand() + xmin;
    othercars.car{2}.pos(1) = xpos;
    if abs(othercars.car{1}.pos(1)-othercars.car{2}.pos(1)) > 10000
       flag = false;
    end 
  end

end 
%------------------------------
    
end


