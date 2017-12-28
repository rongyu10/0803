function ems = plot_axisinfo(axisinfo,varargin)
%
%   unitX      = varargin{1}: Unit length for X-axis plot
%   unitY      = varargin{2}; Unit length for Y-axis plot
%   axis_diffX = varargin{3}; Location for X-axis plot
%   axis_diffY = varargin{4}; Location for Y-axis plot


persistent first_flag
if isempty(first_flag)
    first_flag = true;
end

SHOW_AXIS = 1;

com = computer;
if isequal(com(1:3), 'MAC')
    fs = 24;
else
    fs = 15;
end
col = [0.7 0.9 0.7];

%---------
if (nargin == 5)
   unitX    = varargin{1};
   unitY    = varargin{2};
   axis_diffX = varargin{3};
   axis_diffY = varargin{4};
elseif (nargin == 3)
   unitX    = varargin{1};
   unitY    = varargin{2};
   axis_diffX = 1000;
   axis_diffY = 1000;   
else
   unitX = 10;
   unitY = 7;
   axis_diffX = 1000;
   axis_diffY = 1000;
end

strX = ['X ' num2str(unitX) 'm'];
strY = ['Y ' num2str(unitY) 'm'];
%---------

iclk = clock;
if first_flag
    first_flag = false;
    if SHOW_AXIS
        xmin = axisinfo(1);
        ymin = axisinfo(3);
        plot([xmin xmin], [ymin ymin + 10000], '-', 'LineWidth', 3, 'Color', col);
        plot([xmin xmin + 10000], [ymin ymin], '-', 'LineWidth', 3, 'Color', col);
        text(xmin+8300, ymin-axis_diffX, strX, 'FontSize', fs, 'Color', col ...
            , 'HorizontalAlignment', 'Center')
        
        h = text(xmin-axis_diffY, ymin+8000, strY, 'FontSize', fs, 'Color', col ...
            , 'HorizontalAlignment', 'Center');  % mod by kumano 'Y 10m' --> 'Y 7m'
        set(h, 'rotation', 90)
    end
    axis equal; axis(axisinfo); axis off;
else
    % DO NOTHING
end
ems = etime(clock, iclk)*1000;
