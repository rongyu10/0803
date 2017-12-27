function [speed, theta] = interaction_humanVSmycar(person,sim)


stoptime = person.stoptime;

idx = find(person.rfs_dists==person.rfs_dist);

if (sim.sec - stoptime)<0.8  % 0.4
    speed = 0.0;
    theta = person.pos_save(3);
    
elseif isempty(idx)
    speed = 0.0;
    theta = person.pos_save(3);
else
    speed = person.vel_save(1);    
    %speed = person.vel_save(1)*0.5;
    %theta = person.pos_save(3);
    theta = avoidanceAngle(person);
end

end

function theta = avoidanceAngle(person)


nr_rfs   = person.nr_rfs;
rfs_degs = person.rfs_degs;
rfs_dists= person.rfs_dists;
rfs_dist = person.rfs_dist;

if (rfs_dists(1)==rfs_dist)&&(rfs_dists(nr_rfs)~=rfs_dist)
   theta =  person.pos_save(3) + rfs_degs(1);
elseif (rfs_dists(1)~=rfs_dist)&&(rfs_dists(nr_rfs)==rfs_dist)
   theta =  person.pos_save(3) + rfs_degs(nr_rfs);
else
    theta =  person.pos_save(3);
end

end


