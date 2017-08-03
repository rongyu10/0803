function flag = is_carcrashed2(mycar)

idx_frontrear = (abs(mycar.rfs_degs) < 31) | (abs(mycar.rfs_degs) > 149);
idx_side      = (abs(mycar.rfs_degs) > 31) & (abs(mycar.rfs_degs) < 149);

mindist_frontrear = min(mycar.rfs_dists(idx_frontrear));
mindist_side      = min(mycar.rfs_dists(idx_side));

if mindist_frontrear < 2000  % 2000[mm]
    flag = 1;
elseif mindist_side < 1000   % 1200[mm]
    flag = 1;
else
    flag = 0;
end
