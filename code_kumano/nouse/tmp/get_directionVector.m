
function vec_direction = get_directionVector(pos, theta)

unitVec = [1 0];
c    = cos(theta*pi/180);
s    = sin(theta*pi/180);
rotation = [c -s; s c]';

vec_tmp = unitVec*rotation;
vec_direction =vec_tmp + pos;

end
