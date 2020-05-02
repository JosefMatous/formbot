classdef robot_plot < matlab.mixin.SetGet
    % ROBOT_PLOT Class for plotting a robot
    %
    % Class that draws a robot using a rectangle for its body, two
    % rectangles for wheels, and a line for "hand".
    
    properties
        % X x-coordinate
        x
        % Y y-coordinate
        y
        % THETA orientation
        theta
        % BODYDIM Body dimensions (width - wheel to wheel, height)
        BodyDim
        % WHEELDIM Wheel dimensions (width, length)
        WheelDim
        % HANDLENGTH Length of the "hand"
        HandLength
        % BODYCOLOR Color of the body plot
        BodyColor
        % WHEELCOLOR Color of the wheel plot
        WheelColor
        % HANDCOLOR Color of the hand plot
        HandColor
        % HANDWIDTH Line width of the hand plot
        HandWidth
    end
    
    properties (SetAccess=private)
        % BODY Handle to body graphics
        Body
        % WHEELS Handle to wheel graphics
        Wheels
        % HAND Handle to hand graphics
        Hand
    end
    
    properties (Access=private)
        is_initialized = false;
    end
    
    methods
        function obj = robot_plot(x, y, theta, varargin)
            %ROBOT_PLOT Construct an instance of this class
            ip = inputParser;
            ip.addRequired('x');
            ip.addRequired('y');
            ip.addRequired('theta');
            ip.addOptional('BodyDim', [0.5,0.3]);
            ip.addOptional('WheelDim', [0.2,0.35]);
            ip.addOptional('HandLength', 0.5);
            ip.addOptional('BodyColor', [0.4,0.4,0.4]);
            ip.addOptional('WheelColor', [0.1,0.1,0.1]);
            ip.addOptional('HandColor', [0.1,0.1,0.1]);
            ip.addOptional('HandWidth', 2);
            ip.parse(x, y, theta, varargin{:});
            
            obj.x = ip.Results.x;
            obj.y = ip.Results.y;
            obj.theta = ip.Results.theta;
            obj.BodyDim = ip.Results.BodyDim;
            obj.WheelDim = ip.Results.WheelDim;
            obj.HandLength = ip.Results.HandLength;
            obj.BodyColor = ip.Results.BodyColor;
            obj.WheelColor = ip.Results.WheelColor;
            obj.HandColor = ip.Results.HandColor;
            obj.HandWidth = ip.Results.HandWidth;
            
            obj.is_initialized = true;
            obj.draw
        end
        
        function draw(obj)
            % DRAW (Re)draws the robot
            %
            % Draws the body, the wheels, and the hand.
            body_vertices = robot_plot.rectangle_vertices(obj.x, obj.y, obj.theta, ...
                obj.BodyDim(2), obj.BodyDim(1), 0, 0);
            wheel_offset = (obj.BodyDim(1) + obj.WheelDim(1))/2;
            lwheel_vertices = robot_plot.rectangle_vertices(obj.x, obj.y, obj.theta, ...
                obj.WheelDim(2), obj.WheelDim(1), 0, wheel_offset);
            rwheel_vertices = robot_plot.rectangle_vertices(obj.x, obj.y, obj.theta, ...
                obj.WheelDim(2), obj.WheelDim(1), 0, -wheel_offset);
            hand_x = [obj.x, obj.x + obj.HandLength*cos(obj.theta)];
            hand_y = [obj.y, obj.y + obj.HandLength*sin(obj.theta)];
            
            ax = gca;
            held = ishold(ax);
            obj.Body = fill(body_vertices(1,:), body_vertices(2,:), obj.BodyColor);
            hold(ax, 'on');
            lwheel = fill(lwheel_vertices(1,:), lwheel_vertices(2,:), obj.WheelColor);
            rwheel = fill(rwheel_vertices(1,:), rwheel_vertices(2,:), obj.WheelColor);
            obj.Wheels = [lwheel, rwheel];
            obj.Hand = plot(hand_x, hand_y, 'LineWidth', obj.HandWidth, 'Color', ...
                obj.HandColor);
            if ~held
                hold(ax, 'off')
            end
        end
        
        function update(obj)
            % UPDATE Updates the robot plot.
            %
            % Changes the plotted objects in accordance with the object's
            % current parameters.
            body_vertices = robot_plot.rectangle_vertices(obj.x, obj.y, obj.theta, ...
                obj.BodyDim(2), obj.BodyDim(1), 0, 0);
            wheel_offset = (obj.BodyDim(1) + obj.WheelDim(1))/2;
            lwheel_vertices = robot_plot.rectangle_vertices(obj.x, obj.y, obj.theta, ...
                obj.WheelDim(2), obj.WheelDim(1), 0, wheel_offset);
            rwheel_vertices = robot_plot.rectangle_vertices(obj.x, obj.y, obj.theta, ...
                obj.WheelDim(2), obj.WheelDim(1), 0, -wheel_offset);
            hand_x = [obj.x, obj.x + obj.HandLength*cos(obj.theta)];
            hand_y = [obj.y, obj.y + obj.HandLength*sin(obj.theta)];
            
            if isvalid(obj.Body) && all(isvalid(obj.Wheels)) && isvalid(obj.Hand)
                obj.Body.Vertices = body_vertices';
                obj.Wheels(1).Vertices = lwheel_vertices';
                obj.Wheels(2).Vertices = rwheel_vertices';
                obj.Hand.XData = hand_x;
                obj.Hand.YData = hand_y;
            else
                error(['Error updating robot plot. One or more graphic ',...
                    'objects have been deleted. Use DRAW to draw the ',...
                    'objects again.'])
            end
        end
        
        function set.x(obj, x)
            assert(robot_plot.is_real_scalar(x), 'x must be a real scalar')
            obj.x = x;
            if obj.is_initialized
                obj.update
            end
        end
        
        function set.y(obj, y)
            assert(robot_plot.is_real_scalar(y), 'y must be a real scalar')
            obj.y = y;
            if obj.is_initialized
                obj.update
            end
        end
        
        function set.theta(obj, theta)
            assert(robot_plot.is_real_scalar(theta), 'theta must be a real scalar')
            obj.theta = theta;
            if obj.is_initialized
                obj.update
            end
        end
        
        function set.BodyDim(obj, d)
            assert(robot_plot.is_valid_dimension(d), ['Body dimensions must be',...
                'a two-elements positive vector.'])
            obj.BodyDim = d;
            if obj.is_initialized
                obj.update
            end
        end

        function set.WheelDim(obj, d)
            assert(robot_plot.is_valid_dimension(d), ['Wheel dimensions must be',...
                'a two-elements positive vector.'])
            obj.WheelDim = d;
            if obj.is_initialized
                obj.update
            end
        end

        function set.HandLength(obj, l)
            assert(robot_plot.is_real_positive(l), ['Hand length must be',...
                'a positive scalar.'])
            obj.HandLength = l;
            if obj.is_initialized
                obj.update
            end
        end

        function set.BodyColor(obj, c)
            assert(robot_plot.is_valid_color(c), ['Body color must be',...
                'either a three-element row vector, or a valid color name.'])
            obj.BodyColor = c;
            if obj.is_initialized
                obj.update
            end
        end

        function set.WheelColor(obj, c)
            assert(robot_plot.is_valid_color(c), ['Wheel color must be',...
                'either a three-element row vector, or a valid color name.'])
            obj.WheelColor = c;
            if obj.is_initialized
                obj.update
            end
        end

        function set.HandColor(obj, c)
            assert(robot_plot.is_valid_color(c), ['Hand color must be',...
                'either a three-element row vector, or a valid color name.'])
            obj.HandColor = c;
            if obj.is_initialized
                obj.update
            end
        end

        function set.HandWidth(obj, w)
            assert(robot_plot.is_real_positive(w), ['Hand width must be',...
                'a positive scalar.'])
            obj.HandWidth = w;
            if obj.is_initialized
                obj.update
            end
        end
    end
    
    methods (Static, Access=private)
        function b = is_real_scalar(a)
            b = isscalar(a) && isnumeric(a) && isreal(a);
        end
        
        function b = is_real_positive(a)
            b = isscalar(a) && isnumeric(a) && isreal(a) && a>0;
        end
        
        function b = is_valid_dimension(a)
            b = isvector(a) && isnumeric(a) && isreal(a) && numel(a)==2 ...
                && all(a>0);
        end
        
        function b = is_valid_color(a)
            b = (isrow(a) && isnumeric(a) && isreal(a) && numel(a)==3 ...
                && all(a>=0) && all(a<=1)) || sum(strcmpi({'y','m','c', ...
                'r','g','b','w','k','yellow','magenta','cyan','red', ...
                'green','blue','white','black','none'}, a)) > 0;
        end
        
        function V = rectangle_vertices(x,y,theta,a,b,x_off,y_off)
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            V0 = [-a/2, a/2, a/2, -a/2;
                  -b/2, -b/2, b/2, b/2] + [x_off;y_off];
            V = R*V0 + [x;y];
        end
    end
end

