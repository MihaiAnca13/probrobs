rosinit
%%
% init global coordinates
global coords_x;
global coords_y;
global velcmd;
global vel;
global odom_scan;

% init listeners to ros topics
fwd_scan = rossubscriber('base_scan_0');
left_scan = rossubscriber('base_scan_1');
right_scan = rossubscriber('base_scan_2');
odom_scan = rossubscriber('odom');
[velcmd, vel] = rospublisher('/cmd_vel');

% wait for listeners to connect and start receiving data
pause(0.5);

% set thresholds for range sensors
TH_F = 0.25;
TH_D = 0.33;

% init arrays that store perfect odom
coords_x = [];
coords_y = [];

% this is a state machine, thus initialising state
state = 0;
% last value of range error used for PD controller
last = 0;
while(1)
    % read range sensors values
    scan = fwd_scan.LatestMessage;
    fwd = scan.Ranges(ceil(size(scan.Ranges,1)/2));
    scan = left_scan.LatestMessage;
    left = scan.Ranges(ceil(size(scan.Ranges,1)/2));
    scan = right_scan.LatestMessage;
    right = scan.Ranges(ceil(size(scan.Ranges,1)/2));
    
    % first state deals with keeping the robot straight until a gap is
    % found
    if state == 0
        delta = left - right;
        if fwd > TH_F && left < TH_D && right < TH_D
            vel.Linear.X = 1;
            vel.Angular.Z = delta*0.85+(delta-last)*5;
            last = delta;
        else
            % move to next state and stop motor when gap found
            vel.Linear.X = 0;
            vel.Angular.Z = 0;
            state = 1;
        end
    end
    % second state moves the robot in the middle of the cell and prepares
    % it for next state
    if state == 1
        if fwd <= TH_F
                vel.Linear.X = 0;
                state = 2;
        else
            if left > TH_D || right > TH_D
                find_middle(left_scan, right_scan, TH_D);
                state = 2;
            end
        end
    end
    % third state decides which direction to go next or turns the robot
    % if at a dead end
    if state == 2
        % 2 - right | 1 - left | 0 - fwd
        available_dir = [];
        if fwd > TH_F
            % adding fwd direction three times to increase the chances of
            % the robot going forward
            available_dir = [available_dir 0 0 0];
        end
        if left > TH_D
            available_dir = [available_dir 1];
        end
        if right > TH_D
            available_dir = [available_dir 2];
        end
        if isempty(available_dir)
            turn(2);
            turn(2);
            state = 0;
        else
            % randomly choosing a direction
            dir = datasample(available_dir, 1);
            if dir == 0
                % if fwd go back restart states
                state = 0;
            else
                % else go and turn
                state = 3;
            end 
        end
    end
    if state == 3
        % turn than move fwd for a small distance so the robot sensors are
        % reading the walls and not the gap
        turn(dir);
        nudge();
        state = 0;
    end
    % debugging purposes only
    if state == 4
        vel.Linear.X = 0;
        vel.Angular.Z = 0;
    end
   
    % call function that saves state and sends variable 'vel' to ros based
    % on a delay
    send_cmd(0.1);
end
%% plotting perfect odom
figure(1);
plot(coords_x, coords_y, 'r');
axis([-5 5 -2.5 2.5]);
%% reset robot to init position
[resetclient, resetmsg] = rossvcclient('/reset_positions');
resetclient.call(resetmsg);
%% debugging turning function
turn(1);
%% debugging odometry function
while (1)
    a = get_pose(odom_scan);
    disp(a)
end
%% turns the robot 90 degrees in a direction
function a = turn(dir)
    global velcmd;
    global vel;
    global odom_scan;

    % returns orientation angle of the robot in radians
    w = get_angle(odom_scan);
    % array contains angles when turning left/right from a specific angle
    turn_matrix = [-3.14 -1.57 1.57; 1.57 3.14 0; 0 1.57 -1.57; -1.57 0 -3.14; 3.14 -1.57 1.57];
    
    DELTA = 0.001; % target error
    
    % extracting closest perfect angle and calculating the target angle
    % based on direction of turning
    [val, idx]=min(abs(turn_matrix(:,1)-w));
    target = turn_matrix(idx,dir+1);
    
    % init error variables
    max_err = abs(abs(w)-abs(target));
    last = max_err;
    err = max_err;
    
    vel.Linear.X = 0;
    
    reset_c = 0;
    
    % while target not reached
    while err > DELTA
        % get error
        err = abs(abs(w)-abs(target));
        % check if overshot and dir needs to be reversed
        % reset_c is making sure this is not being called at every loop
        if err > last && reset_c == 4
            if dir == 1
                dir = 2;
            else
                dir = 1;
            end
            reset_c = 0;
        end
        reset_c = reset_c + 1;
        if reset_c > 4
            reset_c = 4;
        end
        % fit error on a curve to smooth the movement
        x = (err/max_err);
        speed = 2.9-exp(1.05-x);
        %speed = 0.02 + x * 3;
        % set vel based on direction
        if dir == 1
            vel.Angular.Z = speed;
        else
            vel.Angular.Z = -speed;
        end
        % update last error
        last = abs(w-target);
        send_cmd(0.1);
        % update angle after sending command and waiting 0.1 secs
        w = get_angle(odom_scan);
    end
    
    % stop robot
    vel.Angular.Z = 0;
    send_cmd(0);
end

% function used for moving fwd until the robot is in the middle of a cell
% when looking at a gap
function b = find_middle(left_scan, right_scan, TH_D)
    global vel;
    global velcmd;

    % read side sensors
    left = left_scan.LatestMessage.Ranges;
    right = right_scan.LatestMessage.Ranges;
    
    % choose which gap is the reference
    if right > left
        target = right;
        scan = right_scan;
    else
        scan = left_scan;
        target = left;
    end

    % set slow speed
    vel.Linear.X = 0.15;
    vel.Angular.Z = 0;
    % based on pre set threshold, go fwd until condition met
    while target >= TH_D
        vel.Linear.X = target-TH_D + 0.01;
        send_cmd(0.1);
        target = scan.LatestMessage.Ranges;
    end
    % no turning needed
    vel.Angular.Z = 0;
end

% function used for moving fwd a small distance
function c = nudge()
    global vel;
    global velcmd;
    
    vel.Linear.X = 0.5;
    vel.Angular.Z = 0;
    send_cmd(0.3);
end

% returns the orientation angle of the robot in radians
function w = get_angle(odom_scan)
    % creates a quaternion (X Y Z W)
    quatern = [odom_scan.LatestMessage.Pose.Pose.Orientation.X odom_scan.LatestMessage.Pose.Pose.Orientation.Y odom_scan.LatestMessage.Pose.Pose.Orientation.Z odom_scan.LatestMessage.Pose.Pose.Orientation.W];
    % Convert quaternion to Euler angles
    eula = quat2eul(quatern);
    % rotation around z axis
    w = eula(3);
end

% returns the pose of the robot
function p = get_pose(odom_scan)
    p = zeros(3,1);
    p(1) = odom_scan.LatestMessage.Pose.Pose.Position.X;
    p(2) = odom_scan.LatestMessage.Pose.Pose.Position.Y;
    p(3) = get_angle(odom_scan);
end

% send vel cmd and save current odometry
function [] = send_cmd(delay)
    global vel;
    global velcmd;
    global odom_scan;
    global coords_x;
    global coords_y;

    send(velcmd, vel);
    pause(delay);
    
    x = odom_scan.LatestMessage.Pose.Pose.Position.X;
    y = odom_scan.LatestMessage.Pose.Pose.Position.Y;
    coords_x = [coords_x x];
    coords_y = [coords_y y];
end