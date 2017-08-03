function flag = is_carclose(person)

mindist = min(person.rfs_dists);

if mindist < 2500  % 2000[mm]
    flag = 1;
else
    flag = 0;
end
