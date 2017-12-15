function degree = get_targetdegree(path, index)
%GET_TARGETDEGREE この関数の概要をここに記述
%   詳細説明をここに記述
if index~=201
    vx= path(index+1,1)-path(index,1);
    vy= path(index+1,2)-path(index,2);
else
    vx= path(index,1)-path(index-1,1);
    vy= path(index,2)-path(index-1,2);
end
degree = atan(vy/vx)*180/pi;

end

