function track = add_track_tollplaza(prev_track, nr_lane, lane_width, nr_seg, lane_length, pointdata,tracktype, varargin)
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
        case {'left_turn' 'left_turn45'}
           startpos = [0, track.width/2, 0];
        case {'right_turn' 'right_turn45'}
           startpos = [0, track.width/2, 0];           
    end
else
    switch lower(tracktype)
        case 'plaza'
           startpos = [prev_track.xmax, 0, 0];
        case 'straight'
           prev_nr_seg  = prev_track.nr_seg;
           prev_type    = prev_track.seg{prev_nr_seg}.type;
           if (strcmp(prev_type,'left_turn'))||(strcmp(prev_type,'left_turn45'))||...
              (strcmp(prev_type,'right_turn'))||(strcmp(prev_type,'right_turn45'))
              startpos = prev_track.seg{prev_nr_seg}.endpos;        
           else
              startpos = [prev_track.xmax, track.width/2, 0];            
           end
        case {'left_turn' 'left_turn45'}
           prev_nr_seg   = prev_track.nr_seg;
           startpos = prev_track.seg{prev_nr_seg}.endpos;
        case {'right_turn' 'right_turn45'}
           prev_nr_seg   = prev_track.nr_seg;
           startpos = prev_track.seg{prev_nr_seg}.endpos;           
    end
end

%--Specifying "startpos" ----------------
if nargin==8
  startpos = varargin{1};
end
%----------------------------------------

seg_length = lane_length/nr_seg;
for iseg =1: nr_seg

  switch lower(tracktype)
    case ''
        % DO NOTHING
    case 'plaza'
        track  = add_segment_tollplaza(track, 'plaza', track.width, seg_length, startpos, pointdata);        
    case 'straight'
        track  = add_segment_tollplaza(track, 'straight', track.width, seg_length, startpos, []);        
    case 'left_turn'
        track  = add_segment_tollplaza(track, 'left_turn', track.width, seg_length, startpos, []);
    case 'left_turn45'
        track  = add_segment_tollplaza(track, 'left_turn45', track.width, seg_length, startpos, []);        
    case 'right_turn'
        track  = add_segment_tollplaza(track, 'right_turn', track.width, seg_length, startpos, []); 
    case 'right_turn45'
        track  = add_segment_tollplaza(track, 'right_turn45', track.width, seg_length, startpos, []);         
    otherwise
        fprintf(2, 'TRACKTYPE [%s] IS NOT DEFINED.n', upper(tracktype));       
  end
end


end
