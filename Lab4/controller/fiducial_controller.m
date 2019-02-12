
global GL_robot_pose;       % global filled by ROS subscriber callback function
global GL_ranges;           % global filled by ROS subscriber callback functions
global GL_markers;          % global filled by ROS subscriber callback function
global GL_NoisyMarkers;
global GL_NumMarkers;       

GL_NumMarkers = 3;                  % number of landmarks in the map
n       = 50;                       % number of iterations
pose    = zeros(3,n);               % storage space for robot pose
range   = zeros(3,n);               % storage space for sensor ranges
marker  = zeros(2,GL_NumMarkers,n); % storage for fiducial observations

% Subscribers to rangers and odometry topics
getrange0 = rossubscriber('base_scan_0', @ROSRange0Callback);
getrange1 = rossubscriber('base_scan_1', @ROSRange1Callback);
getrange2 = rossubscriber('base_scan_2', @ROSRange2Callback);
getodom = rossubscriber('/odom',@ROSPoseCallback);
getmarkers = rossubscriber('base_marker_detection', @ROSMarkerCallback);

% Publisher to velocity command topic
[velcmd, vel] = rospublisher('/cmd_vel');

% closed loop control
for i=1:n
    pause(0.1);     % 100ms delay to match Simulator
    
    error = GL_ranges(2) - GL_ranges(3);    % check central to coridor
    if(GL_ranges(1)<0.2)
        vel.Angular.Z = 3;
    else
    vel.Angular.Z = error*8;                % Adjust rotation if not
    end
    vel.Linear.X = GL_ranges(1)-0.1;        % check front is clear to move
    send(velcmd, vel)                       % publish message to topic
    
    %% log data locally
    pose(:,i) = GL_robot_pose;         % robot pose
    range(:,i) = GL_ranges;             % ranger returns
    marker(:,:,i) = GL_markers;          % fiducial observations
end
%%
i = 1;
marker = zeros(2,GL_NumMarkers,100);
noisy_marker = zeros(2,GL_NumMarkers,100);
while (i < 100)
    marker(:,:,i) = GL_markers;
    noisy_marker(:,:,i) = GL_NoisyMarkers;
    i = i + 1;
    disp(i);
    pause(0.1);
end
%%
figure(1);
clf
plot(marker(1,:), marker(2,:), 'or');
axis([-2.2 2.2 -2 2]);
figure(2);
plot(noisy_marker(1,:), noisy_marker(2,:), 'ob');
axis([-2.2 2.2 -2 2]);
 %% draw a plot of odometry
 figure(1)
 clf
 plot(pose(1,:), pose(2,:),'r'); % Plot the x,y robot position during run

 %% Write data to file
 fileID = fopen('odometry.dat','w');
 fprintf(fileID, '%f %f %f \n', pose);
 fclose(fileID);
 
 fileID = fopen('ranges.dat','w');
 fprintf(fileID, '%f %f %f \n', range);
 fclose(fileID);
 
 fileID = fopen('fiducials.dat','w');
 fprintf(fileID, '%f %f \n', marker);
 fclose(fileID);
 
 

