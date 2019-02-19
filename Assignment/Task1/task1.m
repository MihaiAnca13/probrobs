rosinit
%%
global coords_x;
global coords_y;
global velcmd;
global vel;
global odom_scan;

fwd_scan = rossubscriber('base_scan_0');
left_scan = rossubscriber('base_scan_1');
right_scan = rossubscriber('base_scan_2');
odom_scan = rossubscriber('odom');
[velcmd, vel] = rospublisher('/cmd_vel');

pause(0.5);

TH_F = 0.25;
TH_D = 0.33;

coords_x = [];
coords_y = [];

state = 0;
last = 0;
while(1)
    scan = fwd_scan.LatestMessage;
    fwd = scan.Ranges(ceil(size(scan.Ranges,1)/2));
    scan = left_scan.LatestMessage;
    left = scan.Ranges(ceil(size(scan.Ranges,1)/2));
    scan = right_scan.LatestMessage;
    right = scan.Ranges(ceil(size(scan.Ranges,1)/2));
    
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
                find_middle(left_scan, right_scan, TH_D);
                state = 2;
            end
        end
    end
    if state == 2
        % 2 - right | 1 - left | 0 - fwd
        available_dir = [];
        if fwd > TH_F
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
figure(1);
plot(coords_x, coords_y, 'r');
axis([-5 5 -2.5 2.5]);
%%
[resetclient, resetmsg] = rossvcclient('/reset_positions');
resetclient.call(resetmsg);
%%
turn(1);
%%
while (1)
    a = get_pose(odom_scan);
    disp(a)
end
%%
function a = turn(dir)
    global velcmd;
    global vel;
    global odom_scan;

    w = get_angle(odom_scan);
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
        w = get_angle(odom_scan);
    end
    
    vel.Angular.Z = 0;
    send_cmd(0);
end

function b = find_middle(left_scan, right_scan, TH_D)
    global vel;
    global velcmd;

    left = left_scan.LatestMessage.Ranges;
    right = right_scan.LatestMessage.Ranges;
    
    if right > left
        target = right;
        scan = right_scan;
    else
        scan = left_scan;
        target = left;
    end

    vel.Linear.X = 0.15;
    vel.Angular.Z = 0;
    while target >= TH_D
        vel.Linear.X = target-TH_D + 0.01;
        send_cmd(0.1);
        target = scan.LatestMessage.Ranges;
    end
    vel.Angular.Z = 0;
end

function c = nudge()
    global vel;
    global velcmd;
    
    vel.Linear.X = 0.5;
    vel.Angular.Z = 0;
    send_cmd(0.3);
end

function w = get_angle(odom_scan)
    quatern = [odom_scan.LatestMessage.Pose.Pose.Orientation.X odom_scan.LatestMessage.Pose.Pose.Orientation.Y odom_scan.LatestMessage.Pose.Pose.Orientation.Z odom_scan.LatestMessage.Pose.Pose.Orientation.W];
    eula = quat2eul(quatern);
    w = eula(3);
end

function p = get_pose(odom_scan)
    p = zeros(3,1);
    p(1) = odom_scan.LatestMessage.Pose.Pose.Position.X;
    p(2) = odom_scan.LatestMessage.Pose.Pose.Position.Y;
    p(3) = get_angle(odom_scan);
end

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