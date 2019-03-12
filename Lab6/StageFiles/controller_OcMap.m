
global GL_robot_pose;      % global filled by ROS subscriber callback function
global GL_ranges;          % global filled by ROS subscriber callback functions
%global markers;

n = 800;                % number of iterations
pose = zeros(3,n);      % storage space for robot pose
range = zeros(3,n);     % storage space for sensor ranges

% Subscribers to rangers and odometry topics
getrange0 = rossubscriber('base_scan_0', @ROSRange0Callback);
getrange1 = rossubscriber('base_scan_1', @ROSRange1Callback);
getrange2 = rossubscriber('base_scan_2', @ROSRange2Callback);
getodom = rossubscriber('/odom',@ROSPoseCallback);

%getmarkers = rossubscriber('base_marker_detection', @ROSMarkerCallback);

% Publisher to velocity command topic
[velcmd, vel] = rospublisher('/cmd_vel');

% closed loop control
for i=1:n
    pause(0.1);     % 100ms delay to match Simulator
    
    error = GL_ranges(2) - GL_ranges(3);  % check central to coridor
    vel.Angular.Z = error*8;        % Adjust rotation if not
    vel.Linear.X = GL_ranges(1)-0.1;   % check front is clear to move
    send(velcmd, vel)               % publish message to topic
    pose(:,i) = GL_robot_pose;         % log robot pose
    range(:,i) = GL_ranges;            % log ranges from sensors measurements
    %markers
end

 figure(1)
 clf
 plot(pose(1,:), pose(2,:),'r'); % Plot the x,y robot position during run
%%
 fileID = fopen('../data/odometry.dat','w');
 fprintf(fileID, '%f %f %f \n', pose);
 fclose(fileID)
 
 fileID = fopen('../data/ranges.dat','w');
 fprintf(fileID, '%f %f %f \n', range);
 fclose(fileID)
 
 

