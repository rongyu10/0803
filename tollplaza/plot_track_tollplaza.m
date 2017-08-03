function axisinfo = plot_track_tollplaza(road, FILL_LANES)
persistent first_flag axisinfobu h
if isempty(first_flag)
    first_flag = true;
end

nr_track = road.nr_track;

if first_flag
    first_flag = false;
  
  for itrack = 1:nr_track
    colors = 0.2*ones(road.track{itrack}.nr_seg, 3); % <= Gray lanes
    lanecolor = [1 1 1]*0.0;
    fillcolor = 0.3*[1 1 1];
    lw = 2;
    arrowlen = 5000;
    textoffset = 3500;
    axisinfo = [inf -inf inf -inf];
    for i = fliplr(1:road.track{itrack}.nr_seg)
        % REVERSE ORDERS
        curr_pos   = road.track{itrack}.seg{i}.startpos;
        curr_bd    = road.track{itrack}.seg{i}.bd;
        curr_type  = road.track{itrack}.seg{i}.type;
        curr_color = colors(i, :);
        ps = road.track{itrack}.seg{i}.p;
        % CORNERS
        p1 = ps(1, :); p2 = ps(2, :); p3 = ps(3, :); p4 = ps(4, :);
        
        
        
        % FILL
        if FILL_LANES
            hfill{itrack} = fill(curr_bd(:, 1), curr_bd(:, 2), fillcolor);
            set(hfill{itrack}, 'EdgeColor', 'k', 'LineWidth', 1);
        else
            hfill{itrack} = plot(curr_bd(:, 1), curr_bd(:, 2), 'k', 'LineWidth', 1);
        end
        % LANES
        nr_lane = road.track{itrack}.nr_lane;
        switch curr_type
            case 'straight'
                for j = 1:nr_lane - 1
                    a = j/(nr_lane)*p4 + (nr_lane-j)/(nr_lane)*p1;
                    b = j/(nr_lane)*p3 + (nr_lane-j)/(nr_lane)*p2;
                    xs = [a(1) a(2)]; ys = [b(1) b(2)];
                    plot([a(1) b(1)], [a(2) b(2)], '-', 'Color', lanecolor, 'LineWidth', 1);
                end
            case 'right_turn'
                l1 = road.track{itrack}.seg{i}.l1;
                l2 = flipud(road.track{itrack}.seg{i}.l2);
                for j = 1:nr_lane - 1
                    ltemp = j/(nr_lane)*l1 + (nr_lane-j)/(nr_lane)*l2;
                    xs = ltemp(:, 1); ys = ltemp(:, 2);
                    plot(ltemp(:, 1), ltemp(:, 2), '-', 'Color', lanecolor, 'LineWidth', 1);
                end
            case 'left_turn'
                l1 = road.track{itrack}.seg{i}.l1;
                l2 = flipud(road.track{itrack}.seg{i}.l2);
                for j = 1:nr_lane - 1
                    ltemp = j/(nr_lane)*l1 + (nr_lane-j)/(nr_lane)*l2;
                    plot(ltemp(:, 1), ltemp(:, 2), '-', 'Color', lanecolor, 'LineWidth', 1);
                end
        end
        % Start position
        % plot_arrow(curr_pos(1:2), arrowlen/2, curr_pos(3), curr_color, 3);
        % Boudnary
        % plot(curr_bd(:, 1), curr_bd(:, 2), '-', 'LineWidth', lw, 'Color', curr_color);
        % Center position for turn
        switch curr_type
            case 'straight'
            case 'right_turn'
                curr_centerpos = road.track{itrack}.seg{i}.centerpos;
                if 1
                    plot(curr_centerpos(1), curr_centerpos(2), 'x', 'LineWidth', 2 ...
                        , 'MarkerSize', 15, 'Color', curr_color);
                end
            case 'left_turn'
                curr_centerpos = road.track{itrack}.seg{i}.centerpos;
                if 1
                    plot(curr_centerpos(1), curr_centerpos(2), 'x', 'LineWidth', 2 ...
                        , 'MarkerSize', 15, 'Color', curr_color);
                end
        end
%----------------        
        % Get axis info from 'curr_bd'
%         xs = curr_bd(:, 1);
%         ys = curr_bd(:, 2);
%         xmin = min(xs); xmax = max(xs);
%         ymin = min(ys); ymax = max(ys);
%         if xmin < axisinfo(1)
%             axisinfo(1) = xmin;
%         end
%         if xmax > axisinfo(2)
%             axisinfo(2) = xmax;
%         end
%         if ymin < axisinfo(3)
%             axisinfo(3) = ymin;
%         end
%         if ymax > axisinfo(4)
%             axisinfo(4) = ymax;
%         end

%----------------    
        % PLOT ADDITIONAL LINES FOR DEBUGGING
        DEBUGGING_LINE = 0;
        if DEBUGGING_LINE
            plot(ps([1 3], 1), ps([1 3], 2), 'r-', 'LineWidth', 4);
        end
    end
    
    % TEXT
    for i = fliplr(1:road.track{itrack}.nr_seg)
        curr_pos = road.track{itrack}.seg{i}.startpos;
        text_pos = curr_pos(1:2) + textoffset*[cos(curr_pos(3)*pi/180) sin(curr_pos(3)*pi/180)];
    end
    % START LANE
    if itrack==1
       startp1 = road.track{itrack}.seg{1}.p(1, :);
       startp4 = road.track{itrack}.seg{1}.p(4, :);
       plot([startp1(1) startp4(1)], [startp1(2) startp4(2)], 'r-', 'LineWidth', 4);
       axis equal;
       axisinfobu = axisinfo;
    end
%     % TRAFFIC LIGHT
%     if isfield(track, 'traffic')
%         lane_idx = track.traffic.lane_idx;
%         ps = track.seg{lane_idx}.p;
%         % CORNERS
%         p1 = ps(1, :); p2 = ps(2, :); p3 = ps(3, :); p4 = ps(4, :);
%         % UNIT VECTORS
%         ux = p3-p4; ux = ux / norm(ux);
%         uy = p3-p2; uy = uy / norm(uy);
%         
%         % THREE LOCATIONS
%         xoff = 2000;
%         yoff = 2000;
%         cpos = 0.5*ps(2, :) + 0.5*ps(3, :); % CENTER
%         gpos = cpos + xoff*ux - yoff*uy;
%         ypos = cpos + xoff*ux;
%         rpos = cpos + xoff*ux + yoff*uy;
%         
%         r = 800; n = 20;
%         gbd  = get_circlebd(gpos, r, n);
%         ybd  = get_circlebd(ypos, r, n);
%         rbd  = get_circlebd(rpos, r, n);
%         
%         % PLOT TRAFFIC LIGHTS 
%         traffic_line = ps([2 3], :);
%         plot(traffic_line(:, 1), traffic_line(:, 2), 'w--', 'LineWidth', 4);
%         [gcol, ycol, rcol] = get_trafficcolors(track.traffic.type);
%         h.gfill = fill(gbd(:, 1), gbd(:, 2), gcol);
%         set(h.gfill, 'EdgeColor', 'k', 'LineWidth', 2, 'FaceAlpha', 0.8);
%         h.yfill = fill(ybd(:, 1), ybd(:, 2), ycol);
%         set(h.yfill, 'EdgeColor', 'k', 'LineWidth', 2, 'FaceAlpha', 0.8);
%         h.rfill = fill(rbd(:, 1), rbd(:, 2), rcol);
%         set(h.rfill, 'EdgeColor', 'k', 'LineWidth', 2, 'FaceAlpha', 0.8);
%     end
  end
  
  % Get axis info 
  axisinfo(1) = road.xmin;
  axisinfo(2) = road.xmax;
  axisinfo(3) = road.ymin;
  axisinfo(4) = road.ymax;
  
else
    % DO NOTHING
    axisinfo = axisinfobu;
    % PLOT TRAFFIC LIGHTS
%     if isfield(track, 'traffic')
%         [gcol, ycol, rcol] = get_trafficcolors(track.traffic.type);
%         h.gfill.FaceColor = gcol;
%         h.yfill.FaceColor = ycol;
%         h.rfill.FaceColor = rcol;
%     end
end