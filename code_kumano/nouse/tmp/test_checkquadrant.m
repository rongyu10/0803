v  = [10 0];

v1  = [200 200];
v2  = [-200 200];
v3  = [-200 -200];
v4  = [200 -200];

inner1 = dot(v,v1);
inner2 = dot(v,v2);
inner3 = dot(v,v3);
inner4 = dot(v,v4);

v1_m = [v1 0];
v2_m = [v2 0];
v3_m = [v3 0];
v4_m = [v4 0];

v_m = [v 0];

cross1 = cross(v_m, v1_m);
cross2 = cross(v_m, v2_m);
cross3 = cross(v_m, v3_m);
cross4 = cross(v_m, v4_m);
