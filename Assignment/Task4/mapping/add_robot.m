function [map] = add_robot(robot_pose, map, scale,l_free,l_occ, l_prior, ranges) %

alpha_real = 0.05;
alpha = alpha_real/scale;  % half size of assumed minimum sensed object (m/scale = cells)
robot_rad = 0.1;    % radius of round robot (m)
z_max = 2;          % maximum range of sensor (m)
fov = 10*(pi/180);  % field of view of sensor cone (rad) (Beta)

robot_rad_cells=round(robot_rad/scale); % how many cells wide is the robot
z_max_cells = round(z_max/scale);       % how many cells is the max range of the sensor

x_cen = robot_pose(1); % get centre coordinates
y_cen = robot_pose(2); % get centre coordinates
theta = robot_pose(3); % get orientation

% round body occupies space so must be free space
for x=(round(x_cen)-robot_rad_cells):(round(x_cen)+robot_rad_cells)
	for y=(round(y_cen)-robot_rad_cells):(round(y_cen)+robot_rad_cells)
		r=sqrt((x_cen-x)^2 + (y_cen-y)^2);
		if r<robot_rad_cells        %check that L2 distance fits radius of Robot
			% update the occupancy of each cell
			map(x,y)=fuse(map(x,y), l_free, l_prior);    
		end
	end
end

% pose of each of the 3 rangers (taken from PlayerStage model)
% updated to 45 degrees sensors
sensor_pose_R = [robot_rad_cells     0                 0;
                0           robot_rad_cells -robot_rad_cells;
                0               pi/4                -pi/4    ];  

% add inverse sensor model
for k=1:length(ranges)          % for each sensor measurement in observation
	[index sensor_pose_G] = get_perceptual_field_sq(sensor_pose_R(:,k), robot_pose, fov, z_max+alpha_real, scale); % find perceptual window
	
	nothing_detected = ranges(k) == z_max; % add exception if no range returned
	
	range = ranges(k)/scale;        % convert range from metres into cells
	for i=index(1):index(2)             % } cycle through each cell in region
		for j=index(3):index(4)     % }
			
			[r phi] = cart2pol(sensor_pose_G(1:2),[i j]);   % get polar coordinates of cell from sensor base
			 
			if cos(phi-sensor_pose_G(3)) > cos(fov/2)	% check that bearing to cell is within the fov of the sensor
				
				if nothing_detected
					if (r < range)           % check that cell is closer than range returned by sensor
						map(i,j)=fuse(map(i,j),l_free, l_prior); % if it is then fuse with l_free
                    end
                    
                else
					if (r < range-(alpha))           % check that cell is closer than range returned by sensor
						map(i,j)=fuse(map(i,j),l_free, l_prior); % if it is then fuse with l_free
					elseif (r >= range-(alpha) && r < range+(alpha)) % if cell is within range +/- alpha
						map(i,j)=fuse(map(i,j),l_occ, l_prior); % ...then fuse with l_occ
                    end
                    
                end     % else nothing_detected not true
            end         % if bearing within perceptual field
        end             % for y index of perceptual field
    end                 % for x index of perceptual field
end                     % for each sensor in ranger

