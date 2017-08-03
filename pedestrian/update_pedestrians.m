function pedestrians = update_pedestrians(pedestrians,mycar,sim)

for i = 1:pedestrians.n

    flag_close = is_carclose(pedestrians.person{i});
    if flag_close
        if isempty(pedestrians.person{i}.stoptime)
            pedestrians.person{i}.stoptime = sim.sec;
            pedestrians.person{i}.vel_save = pedestrians.person{i}.vel;
            pedestrians.person{i}.pos_save = pedestrians.person{i}.pos;
            pedestrians.person{i}.noobstacletime = [];
        end
        
        [speed, theta] = interaction_humanVSmycar(pedestrians.person{i},sim);
        pedestrians.person{i}.vel(1) = speed;
        pedestrians.person{i}.pos(3) = theta;
    elseif isempty(pedestrians.person{i}.noobstacletime)
        pedestrians.person{i}.noobstacletime = sim.sec;
        %pedestrians.person{i}.vel(1) = pedestrians.person{i}.vel_save(1);
        %pedestrians.person{i}.pos(3) = pedestrians.person{i}.pos_save(3);
        %pedestrians.person{i}.stoptime =[];
    elseif ((sim.sec - pedestrians.person{i}.noobstacletime) > 1.0) &&(~isempty(pedestrians.person{i}.stoptime))
        pedestrians.person{i}.vel(1) = pedestrians.person{i}.vel_save(1);
        pedestrians.person{i}.pos(3) = pedestrians.person{i}.pos_save(3);        
        pedestrians.person{i}.stoptime =[];
    end
    
    %{
    %  OLD version
    flag_close = is_carclose(pedestrians.person{i});
    if flag_close
        pedestrians.person{i}.vel(1) = 0.0;
    else
        pedestrians.person{i}.vel(1) = pedestrians.person{i}.vel_save(1);
    end
    %}
    
    %--- update--------
    pedestrians.person{i}.pos ...
        = update_pos(pedestrians.person{i}.pos, pedestrians.person{i}.vel, sim.T); 
    pedestrians.person{i}.bd ... 
        = get_personshape(pedestrians.person{i}.pos, pedestrians.person{i}.D);
    pedestrians = update_pedestrian_rfs(pedestrians, mycar);
    %------------------
end

end

function pedestrians = update_pedestrian_rfs(pedestrians, mycar)


for i = 1:pedestrians.n
    pos1 = pedestrians.person{i}.pos(1:2);
    pos2 = mycar.pos(1:2);
    tmp_dist = norm(pos2 - pos1);
    if tmp_dist < pedestrians.person{i}.rfs_dist
       pedestrians.person{i}.rfs_dists = calc_pedestrian_rfs(pedestrians.person{i}, mycar); 
    else
       pedestrians.person{i}.rfs_dists = pedestrians.person{i}.rfs_dist*ones(1, pedestrians.person{i}.nr_rfs); 
    end
end

end

function rfs_dists = calc_pedestrian_rfs(person, mycar)

%person.r = 0;
% EMULATE RFS
D2R    = pi/180;
nr_rfs = person.nr_rfs;
result = person.rfs_dist*ones(1, nr_rfs);

for i = 1:nr_rfs
    % FOR ALL RANGEFINDER BEAMS
    rfs_deg = person.pos(3) + person.rfs_degs(i);
    rfs_rad = rfs_deg * D2R;
    rfs_fr  = person.pos(1:2);
    rfs_to  = person.pos(1:2) ...
        + (person.r+person.rfs_dist)*[cos(rfs_rad) sin(rfs_rad)];
    min_dist2obs = person.rfs_dist;
    obs_boundary = zeros(2, 1E3);
    
    nr_obs_b = 0;
    % FOR ALL OBSTACLES(mycar)
    curr_obs_boundary = mycar.bd;
    curr_obs_boundary = [curr_obs_boundary' [NaN ; NaN]];
    nr_curr_obs_b = size(curr_obs_boundary, 2);
    obs_boundary(:, nr_obs_b+1:nr_obs_b + nr_curr_obs_b) = curr_obs_boundary;
    nr_obs_b = nr_obs_b + nr_curr_obs_b;
    obs_boundary = obs_boundary(:, 1:nr_obs_b);
    
    % Use InterX
    int_xy = InterX([rfs_fr(1) rfs_to(1) ; rfs_fr(2), rfs_to(2)], obs_boundary);
    if ~isempty(int_xy)
        xi = int_xy(1, :);
        yi = int_xy(2, :);
        for k = 1:length(xi)
            dist = norm([ mycar.pos(1)-xi(k), mycar.pos(2)-yi(k) ]);
            dist = dist - mycar.r;
            if dist < min_dist2obs
                min_dist2obs = dist;
            end
        end
    end
    result(i) = min_dist2obs;
end
rfs_dists = result;

end