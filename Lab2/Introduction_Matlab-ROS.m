%% Initialise ROS
% create a ROS master or connect to an existing ROS master
% can pass ip address of external master if not local: rosinit('192.168.10.1')
% searches for ROS_MASTER_URI environment variable if no arguments given
rosinit

%% Get all  nodes that are part of the ROS network
rosnode list

%% Get a list of all topics that are available
rostopic list

%% Get info about /odom topic
rostopic info /base_pose_ground_truth

%% See data published on a topic
odometry = rostopic('echo', '/base_pose_ground_truth')
showdetails(odometry)

%% Get information about messages published through the /base_scan
rostopic info /base_scan_0
rosmsg show sensor_msgs/LaserScan

%% Create a subscriber for the /base_scan topic
scanner = rossubscriber('base_scan_0')

%% Three ways of Accessing Data
%% 1. Get the next message that arrives (5 is the timeout)
laserdata = receive(scanner,5)
%% 2. Get the latest data that was received (might be empty)
laserdata = scanner.LatestMessage
%% 3. Set an assynchronous callback for new messages
scanner = rossubscriber('/base_scan_0', @ROSRange0Callback)
%% Delete callback
scanner = [];

%% Visualise laser scan
% Some of the most common messages encountered in robotics such as
% compressed images, laser scans, point clouds, ...) have functions that
% read or visualise
figure;
plot(laserdata, 'MaximumRange', 4) % set maximum range to show at 4m
%% Or, overload the function to generate a plot of LatestMessage
plot(scanner.LatestMessage, 'MaximumRange', 5)

%% Display a ROS image message in MATLAB


%% Create a publisher to control the robot [velcmd = object, vel = message]
[velcmd, vel] = rospublisher('/cmd_vel')

%% Drive forward and turn to the left
vel.Linear.X = 1;
vel.Angular.Z = 0.0;
send(velcmd, vel)

%% Stop the robot
vel.Linear.X = 0;
vel.Angular.Z = 0;
send(velcmd, vel)

%% Drive the robot forward until it is 1 metre from the wall
dist2wall = inf;
while(dist2wall >=1)
    scan = scanner.LatestMessage;
    dist2wall = scan.Ranges(ceil(size(scan.Ranges,1)/2));
    
    if isnan(dist2wall)
        dist2wall = inf;
    end
    
    % Drive forward
    vel.Linear.X = 0.5;
    send(velcmd,vel)
    pause(0.1)
end
dist2wall

% Stop the robot
vel.Linear.X = 0;
send(velcmd,vel)

%% Get all available services
rosservice list

%% Reset the simulation
[resetclient, resetmsg] = rossvcclient('/reset_positions')
resetclient.call(resetmsg)

%% disconnect from ROS
rosshutdown


