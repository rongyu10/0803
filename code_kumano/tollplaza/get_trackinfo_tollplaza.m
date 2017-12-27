function [info, emsec] = get_trackinfo_tollplaza(road, car_pos, othercars)

iclk = clock;
maxdist2car = 40*1000; % <= maximum distance we will care about (40m)

track_idx = -1; % kumano
seg_idx   = -1;

for itrack = 1:road.nr_track % kumano
for i = 1:road.track{itrack}.nr_seg
    curr_bd = road.track{itrack}.seg{i}.bd;
    if inpolygon(car_pos(1), car_pos(2), curr_bd(:, 1), curr_bd(:, 2))
        track_idx = itrack; 
        seg_idx   = i;
        break;
    end
end
end

info.track_idx = track_idx; % kumano
info.seg_idx   = seg_idx;

info.center_fb_dists = maxdist2car;
info.left_fb_dists   = maxdist2car;
info.right_fb_dists  = maxdist2car;

% If car_pos is inside the track,
% 1) compute dist from center / 2) compute geodesic dist
if seg_idx > 0
    seg = road.track{track_idx}.seg{seg_idx};  % kumano
    info.seg = seg;
    info.type = seg.type;
    info.startpos = seg.startpos;

    prev_dist = 0; 
    %-------kumano--- 
    for i = 1:track_idx - 1  
       for j = 1:road.track{i}.nr_seg       
          prev_dist = prev_dist + road.track{i}.seg{j}.len;
       end
    end    
    
    for i = 1:seg_idx - 1
        prev_dist = prev_dist + road.track{track_idx}.seg{i}.len;
    end
    %---------------
    switch info.type
        case 'straight'
            % deviation
            a = seg.startpos(1:2);
            b = seg.endpos(1:2);
            x = car_pos(1:2);
            info.dev = get_distfrpnt2line(x, a, b);
            % distance
            a = seg.p(1, :);
            b = seg.p(4, :);
            info.dist = prev_dist + get_distfrpnt2line(x, a, b);
            % degree
            car_d = car_pos(3);
            seg_d = seg.startpos(3);
            info.deg = mod(car_d - seg_d + 180, 360) - 180;
        case 'plaza'
            % deviation
            a = seg.startpos(1:2);
            b = seg.endpos(1:2);
            x = car_pos(1:2);
            info.dev = get_distfrpnt2line(x, a, b);
            % distance
            a = seg.p(1, :);
            b = seg.p(4, :);
            info.dist = prev_dist + get_distfrpnt2line(x, a, b);
            % degree
            car_d = car_pos(3);
            seg_d = seg.startpos(3);
            info.deg = mod(car_d - seg_d + 180, 360) - 180;            
        case {'right_turn' 'right_turn45'}
            org = seg.centerpos(1:2);
            pntA = seg.startpos(1:2);
            pntB = car_pos(1:2);
            deg_diff = get_degbtwlines(org, pntA, pntB);
            info.deg = mod(seg.startpos(3) + 90 + deg_diff, 360); % wrt seg.centerpos
            info.r   = norm(org - car_pos(1:2));
            info.dev = seg.d - info.r;
            % distance
            curr_dist = 2*pi*seg.d*abs(deg_diff)/360;
            info.dist = prev_dist + curr_dist;
            % degree
            car_d = car_pos(3);
            seg_d = seg.startpos(3) - abs(deg_diff);
            info.deg = mod(car_d - seg_d + 180, 360) - 180;
        case {'left_turn' 'left_turn45'}
            org = seg.centerpos(1:2);
            pntA = seg.startpos(1:2);
            pntB = car_pos(1:2);
            deg_diff = get_degbtwlines(org, pntA, pntB);
            info.deg = mod(seg.startpos(3) - 90 + deg_diff, 360); % wrt seg.centerpos
            info.r   = norm(org - car_pos(1:2));
            info.dev = info.r - seg.d;
            % distance
            info.dist = prev_dist + 2*pi*seg.d*abs(deg_diff)/360;
            % degree
            car_d = car_pos(3);
            seg_d = seg.startpos(3) + abs(deg_diff);
            info.deg = mod(car_d - seg_d + 180, 360) - 180;
    end
    
    % Lane idx
    nr_lane = road.track{track_idx}.nr_lane;
    width = seg.width;
    dev = info.dev;
    unit_len = width / nr_lane;
    lane_idx = ceil( (dev+width/2) / unit_len);
    info.lane_idx = lane_idx;
    
    % Lane deviation
    temp = info.dev + width/2;
    temp2 = mod(temp, unit_len);
    temp3 = temp2 - unit_len/2;
    info.lane_dev = temp3;
    
    if nargin == 3  % <= This is for getting info of mycar!!
        % To other cars
        curr_lane = info.lane_idx;
        left_lane = curr_lane  - 1;
        right_lane = curr_lane + 1;
        cars_in_left_dists   = zeros(100, 1); num_in_left_dists = 0;
        cars_in_left_idxs    = zeros(100, 1);
        cars_in_right_dists  = zeros(100, 1); num_in_right_dists = 0;
        cars_in_right_idxs   = zeros(100, 1);
        cars_in_center_dists = zeros(100, 1); num_in_center_dists = 0;
        cars_in_center_idxs  = zeros(100, 1);
        
        total_track_dist = 0;
        for i = 1:road.nr_track
            for j = 1:road.track{i}.nr_seg
                total_track_dist = total_track_dist + road.track{i}.seg{j}.len;
            end
        end
        
        for i = 1:othercars.n
            % For all other cars, check which lane they are at
            othercarpos  = othercars.car{i}.pos; 
            dist2car = norm(othercarpos(1:2) - car_pos(1:2));
            if dist2car > maxdist2car
                continue;
            end
            
            othercarinfo = get_trackinfo_tollplaza(road, othercarpos); % <= recursive function / not that readable.. mod by kumano
            othercarlane = othercarinfo.lane_idx;
            othercardist = othercarinfo.dist;
            dist_diff    = othercardist  - info.dist ;
            dist_diff2 = mod(dist_diff + total_track_dist/2, total_track_dist) ...
                - total_track_dist/2;
            switch othercarlane
                case curr_lane
                    num_in_center_dists = num_in_center_dists + 1;
                    cars_in_center_idxs(num_in_center_dists) = i;
                    cars_in_center_dists (num_in_center_dists) = dist_diff2;
                case left_lane
                    num_in_left_dists = num_in_left_dists + 1;
                    cars_in_left_idxs(num_in_left_dists) = i;
                    cars_in_left_dists (num_in_left_dists) = dist_diff2;
                case right_lane
                    num_in_right_dists = num_in_right_dists + 1;
                    cars_in_right_idxs(num_in_right_dists) = i;
                    cars_in_right_dists (num_in_right_dists) = dist_diff2;
            end
        end
        cars_in_center_dists = cars_in_center_dists(1:num_in_center_dists);
        cars_in_left_dists   = cars_in_left_dists(1:num_in_left_dists);
        cars_in_right_dists  = cars_in_right_dists(1:num_in_right_dists);
        
        % Center lane
        temp = cars_in_center_dists;
        temp(temp <= 0) = inf;
        centerfw = min(min(temp), maxdist2car);
        temp = cars_in_center_dists;
        temp(temp >= 0) = -inf;
        centerbw = min(min(-temp), maxdist2car);
        
        % Left lane
        temp = cars_in_left_dists;
        temp(temp < 0) = inf;
        leftfw = min(min(temp), maxdist2car);
        temp = cars_in_left_dists;
        temp(temp > 0) = -inf;
        leftbw = min(min(-temp), maxdist2car);
        
        % Right lane
        temp = cars_in_right_dists;
        temp(temp < 0) = inf;
        rightfw = min(min(temp), maxdist2car);
        temp = cars_in_right_dists;
        temp(temp > 0) = -inf;
        rightbw = min(min(-temp), maxdist2car);
        
        if isempty(centerfw), centerfw = maxdist2car; end;
        if isempty(centerbw), centerbw = maxdist2car; end;
        if isempty(leftfw), leftfw = maxdist2car; end;
        if isempty(leftbw), leftbw = maxdist2car; end;
        if isempty(rightfw), rightfw = maxdist2car; end;
        if isempty(rightbw), rightbw = maxdist2car; end;
        
        
        info.center_fb_dists = [centerfw centerbw];
        info.left_fb_dists   = [leftfw leftbw];
        info.right_fb_dists  = [rightfw rightbw];
    end
else
    info.lane_idx = -1;
    info.dev  = nan;
    info.dist = nan;
    info.deg  = nan;
    info.lane_dev = nan;
    
    info.center_fb_dists = [maxdist2car maxdist2car];
    info.left_fb_dists = [maxdist2car maxdist2car];
    info.right_fb_dists = [maxdist2car maxdist2car];
end

% Handling Left-most and Right-most lanes
if track_idx > 0  % kumano
    if info.lane_idx == 1
        % Leftmost
        info.left_fb_dists = [0 0];
    elseif info.lane_idx == road.track{track_idx}.nr_lane
        % Rightmost
        info.right_fb_dists = [0 0];
    end
end
% TRAFFIC LIGHT INFORMATION
%if isfield(track, 'traffic')
%    info = get_tlinfo(info, track);
%end

% Time
emsec = etime(clock, iclk)*1000;
info.emsec = emsec;




