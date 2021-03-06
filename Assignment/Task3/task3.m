rosinit
%%
global coords_x;
global coords_y;
global estimate_x;
global estimate_y;
global estimate_pose;
global velcmd;
global vel;
global robot_pose;
global fwd;
global right;
global left;
global NumMarkers;
global Features;
global landmarks;

landmarks = [-4.148 2.02; 0.652 2; 3.278 0.348];

fwd_scan = rossubscriber('base_scan_0', @FwdCallback);
left_scan = rossubscriber('base_scan_1', @LeftCallback);
right_scan = rossubscriber('base_scan_2', @RightCallback);
markers_scan = rossubscriber('base_marker_detection', @MarkerCallback);
rossubscriber('odom', @OdomCallback);
[velcmd, vel] = rospublisher('/cmd_vel');

pause(0.5);

TH_F = 0.25;
TH_D = 0.33;

coords_x = [];
coords_y = [];
estimate_x = [];
estimate_y = [];
NumMarkers = 1;
robot_pose = [0 0 0];
pause(0.5);
estimate_pose = robot_pose;

state = 0;
last = 0;
while(1)    
    if state == 0
        delta = left - right;
        if fwd > TH_F && left < TH_D && right < TH_D
            vel.Linear.X = 1;
            vel.Angular.Z = delta*0.85+(delta-last)*5;
            last = delta;
        else
            vel.Linear.X = 0;
            vel.Angular.Z = 0;
            state = 1;
        end
    end
    if state == 1
        if fwd <= TH_F
                vel.Linear.X = 0;
                state = 2;
        else
            if left > TH_D || right > TH_D
                find_middle(TH_D);
                state = 2;
            end
        end
    end
    if state == 2
        % 2 - right | 1 - left | 0 - fwd
        available_dir = [];
        if fwd > TH_F
            available_dir = [available_dir 0];
        end
        if left > 0.28
            available_dir = [available_dir 1];
        end
        if right > 0.28
            available_dir = [available_dir 2];
        end
        if isempty(available_dir)
            turn(2);
            turn(2);
            state = 0;
        else
            dir = datasample(available_dir, 1);
            if dir == 0
                state = 0;
            else
                state = 3;
            end 
        end
    end
    if state == 3
        turn(dir);
        nudge();
        state = 0;
    end
    if state == 4
        vel.Linear.X = 0;
        vel.Angular.Z = 0;
    end
   
    send_cmd(0.1);
end
%%
figure;
subplot(2,1,1);
plot(coords_x, coords_y, 'r');
hold on;
plot(landmarks(:,1), landmarks(:,2), 'o');
hold off;
axis([-5 5 -2.5 2.5]);
subplot(2,1,2);
plot(estimate_x, estimate_y, 'b');
hold on;
plot(landmarks(:,1), landmarks(:,2), 'o');
hold off;
axis([-5 5 -2.5 2.5]);
%%
[resetclient, resetmsg] = rossvcclient('/reset_positions');
resetclient.call(resetmsg);
%%
turn(0);
%%
function a = turn(dir)
    global vel;
    global robot_pose;
 
    w = robot_pose(3);
    turn_matrix = [-3.14 -1.57 1.57; 1.57 3.14 0; 0 1.57 -1.57; -1.57 0 -3.14; 3.14 -1.57 1.57];

    %turn_matrix = [-1 -0.7071 0.7071; 0.7071 1 0; 0 0.7071 -0.7071; -0.7071 0 -1; 1 -0.7071 0.7071];
    
    %w = odom_scan.LatestMessage.Pose.Pose.Orientation.Z;
    
    DELTA = 0.001;
    
    [val, idx]=min(abs(turn_matrix(:,1)-w));
    target = turn_matrix(idx,dir+1);
    
    max_err = abs(abs(w)-abs(target));
    last = max_err;
    err = max_err;
    
    vel.Linear.X = 0;
    
    reset_c = 0;
    
    while err > DELTA
        err = abs(abs(w)-abs(target));
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
        x = (err/max_err);
        speed = 2.9-exp(1.05-x);
        %speed = 0.02 + x * 3;
        if dir == 1
            vel.Angular.Z = speed;
        else
            vel.Angular.Z = -speed;
        end
        last = abs(w-target);
        send_cmd(0.1);
        w = robot_pose(3);
    end
    
    vel.Angular.Z = 0;
    send_cmd(0);
end

function b = find_middle(TH_D)
    global vel;
    global left;
    global right;
    
    if right > left
        target = right;
        scan = 0;
    else
        scan = 1;
        target = left;
    end

    vel.Linear.X = 0.15;
    vel.Angular.Z = 0;
    while target >= TH_D
        vel.Linear.X = target-TH_D + 0.01;
        send_cmd(0.1);
        if scan == 0
            target = right;
        else
            target = left;
        end
    end
    vel.Angular.Z = 0;
end

function c = nudge()
    global vel;
    
    vel.Linear.X = 0.5;
    vel.Angular.Z = 0;
    send_cmd(0.3);
end

function [] = send_cmd(delay)
    global vel;
    global velcmd;
    global robot_pose;
    global coords_x;
    global coords_y;
    global estimate_x;
    global estimate_y;
    global estimate_pose;
  
    send(velcmd, vel);
    pause(delay);
    
    x = robot_pose(1);
    y = robot_pose(2);
    
    estimate_pose = vel_model(vel, estimate_pose, 0.1);
    estimate_x = [estimate_x estimate_pose(1)];
    estimate_y = [estimate_y estimate_pose(2)];
    
    coords_x = [coords_x x];
    coords_y = [coords_y y];
end