function ems = plot_pedestrians(pedestrians, SIMPLECARSHAPE, REALCARSHAPE,PLOT_RFS, PLOT_VELOCITY)
persistent fist_flag h
if isempty(fist_flag)
    fist_flag = true;
end

iclk = clock;
%{
if PLOT_RFS
    % ROBOT RFS BOUNDARIES
    rfs_bd = zeros(pedestrians.person{1}.nr_rfs*3, 2);
    %for i = 1:pedestrians.n
    for rfsbdi = 1:pedestrians.person{1}.nr_rfs
        rfs_deg = pedestrians.person{1}.pos(3) + pedestrians.person{1}.rfs_degs(rfsbdi);
        rfs_fr  = pedestrians.person{1}.pos(1:2);
        rfs_to  = rfs_fr + (pedestrians.person{1}.r+pedestrians.person{1}.rfs_dists(rfsbdi)) ...
            *[cos(rfs_deg*pi/180) sin(rfs_deg*pi/180)];
        rfs_bd(3*(rfsbdi-1)+1:3*rfsbdi, :) ...
            = [rfs_fr(1) rfs_fr(2); rfs_to(1) rfs_to(2); nan nan];
    end
    %end
end
%}

if PLOT_RFS
    % ROBOT RFS BOUNDARIES
    nperson = pedestrians.n;
    rfs_bd  = zeros(pedestrians.person{1}.nr_rfs *3 *nperson, 2);
    for i = 1:nperson
        nr_rfs = pedestrians.person{i}.nr_rfs;
        for rfsbdi = 1:nr_rfs
            rfs_deg = pedestrians.person{i}.pos(3) + pedestrians.person{i}.rfs_degs(rfsbdi);
            rfs_fr  = pedestrians.person{i}.pos(1:2);
            rfs_to  = rfs_fr + (pedestrians.person{i}.r+pedestrians.person{i}.rfs_dists(rfsbdi)) ...
                *[cos(rfs_deg*pi/180) sin(rfs_deg*pi/180)];
            i_start = 3*(rfsbdi-1) + 3*nr_rfs*(i-1) + 1;
            i_end   = 3*rfsbdi + 3*nr_rfs*(i-1);
            rfs_bd(i_start:i_end, :) ...
                = [rfs_fr(1) rfs_fr(2); rfs_to(1) rfs_to(2); nan nan];
        end
    end
end



% PLOT
if fist_flag
    fist_flag = false;    
    for i = 1:pedestrians.n
        col = [0 0 1];
        %if SIMPLECARSHAPE
         if 1   
            h.circle{i}     = fill(pedestrians.person{i}.bd.circle(:, 1), pedestrians.person{i}.bd.circle(:, 2),'c');  
            h.personfill{i} = fill(pedestrians.person{i}.bd.triangle(:, 1), pedestrians.person{i}.bd.triangle(:, 2), 'b');
            
        end        
    end
        
    % RFS
    if PLOT_RFS
        h.rfs_bd = plot(rfs_bd(:, 1), rfs_bd(:, 2), '-' ...
            , 'Color', [0.95 0.6 0.4], 'LineWidth', 1);
        h.rfs_bd.Color(4) = 0.5;
    end

if PLOT_VELOCITY
    % Velocity
    for i = 1:pedestrians.n
        box2 = get_searchBOX(pedestrians,i, 4.0);
        h.velocity{i} =  plot(box2(1,:),box2(2,:),'-g');
    end
end
    
else
    % update Pedestrian plot
    for i = 1:pedestrians.n
        %if SIMPLECARSHAPE
        if 1   
            h.circle{i}.Vertices     = pedestrians.person{i}.bd.circle; 
            h.personfill{i}.Vertices = pedestrians.person{i}.bd.triangle;
        end
        
    end
    
    % RFS
    if PLOT_RFS
        h.rfs_bd.XData = rfs_bd(:, 1);
        h.rfs_bd.YData = rfs_bd(:, 2);
    end

if PLOT_VELOCITY
    % Velocity
    for i = 1:pedestrians.n
        box2 = get_searchBOX(pedestrians,i, 4.0);
        h.velocity{i}.XData = box2(1,:);
        h.velocity{i}.YData = box2(2,:);
    end
end
    
end
ems = etime(clock, iclk)*1000;

end

function box = get_searchBOX(pedestrians,idx,evalParam)
% get searchigBOX made by future position of pedestrians

dt = evalParam;
one = ones(1,5);

pos  = pedestrians.person{idx}.pos(1:2);
theta= pedestrians.person{idx}.pos(3);
R    = pedestrians.person{idx}.D/2 + 1250;        
path = pedestrians.person{idx}.vel(1)*dt*1.1;
tmpbox  = [0, path, path, 0, 0; -R, -R, R, R, -R];
c    = cos(theta*pi/180);
s    = sin(theta*pi/180);
rotation = [c -s; s c];
    
box = rotation*tmpbox;
box(1,:) = box(1,:)+ pos(1)*one;
box(2,:) = box(2,:)+ pos(2)*one;

end
