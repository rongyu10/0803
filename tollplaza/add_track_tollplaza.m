function track = add_track_tollplaza(prev_track, nr_lane, lane_width, nr_seg, lane_length, pointdata,tracktype)
% INITIALIZE TRACK
max_seg = 100;
track.nr_seg     = 0;
track.nr_lane    = nr_lane;
track.lane_width = lane_width;
track.width      = lane_width*nr_lane;
track.seg = cell(max_seg, 1);
for i = 1:max_seg
    track.seg{i}.type = '';           % straight, right_turn, left_turn
    track.seg{i}.width = 0;           % width [mm]
    track.seg{i}.startpos = [0 0 0];  % start position
    track.seg{i}.endpos = [0 0 0];    % end position
    track.seg{i}.d = 0;               % straight: segment distance / turn: radius
    track.seg{i}.bd = [];             % boundary
end
track.xmin = inf;
track.ymin = inf;
track.xmax = -inf;
track.ymax = -inf;

% ADD LANES
if isempty(prev_track)
    switch lower(tracktype)
        case 'plaza'
           startpos = [0, 0, 0];
        case 'straight'
           startpos = [0, track.width/2, 0];
    end
else
    switch lower(tracktype)
        case 'plaza'
           startpos = [prev_track.xmax, 0, 0];
        case 'straight'
           startpos = [prev_track.xmax, track.width/2, 0];
    end    
end

seg_length = lane_length/nr_seg;
for iseg =1: nr_seg

  switch lower(tracktype)
    case ''
        % DO NOTHING
    case 'plaza'
        track  = add_segment_tollplaza(track, 'plaza', track.width, seg_length, startpos, pointdata);        
    case 'straight'
        track  = add_segment_tollplaza(track, 'straight', track.width, seg_length, startpos, []);        
    otherwise
        fprintf(2, 'TRACKTYPE [%s] IS NOT DEFINED.n', upper(tracktype));       
  end
end


end
