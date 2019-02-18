vel.Linear.X = 0;
vel.Angular.Z = 3;
p = get_pose();
c = 30;
send(velcmd, vel);
tic;
while c > 0
    last_p = p;
    p = get_pose();
    if sum(p==last_p) < 3
        c = 30;
    else
        if c == 30
            delta = toc;
        end
        c = c-1;
    end
end
delta
%%
send(velcmd, vel);
while (1)
    p = get_pose()
    pause(0.001);
end
%%
[resetclient, resetmsg] = rossvcclient('/reset_positions');
resetclient.call(resetmsg);
%%
function p = get_pose()
    global odom_scan
    p = zeros(3,1);
    p(1) = odom_scan.LatestMessage.Pose.Pose.Position.X;
    p(2) = odom_scan.LatestMessage.Pose.Pose.Position.Y;
    p(3) = get_angle();
end

function w = get_angle()
    global odom_scan
    quatern = [odom_scan.LatestMessage.Pose.Pose.Orientation.X odom_scan.LatestMessage.Pose.Pose.Orientation.Y odom_scan.LatestMessage.Pose.Pose.Orientation.Z odom_scan.LatestMessage.Pose.Pose.Orientation.W];
    eula = quat2eul(quatern);
    w = eula(3);
end

function a = turn(dir)
    global velcmd;
    global vel;
    global odom_scan;

    w = get_angle();
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
        send(velcmd, vel);
        w = get_angle();
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