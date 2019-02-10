rosinit
%%
fwd_scan = rossubscriber('base_scan_0');
left_scan = rossubscriber('base_scan_1');
right_scan = rossubscriber('base_scan_2');
odom_scan = rossubscriber('odom');
[velcmd, vel] = rospublisher('/cmd_vel');

pause(0.5);

TH_F = 0.2;
TH_D = 0.33;

state = 0;
while(1)
    scan = fwd_scan.LatestMessage;
    fwd = scan.Ranges(ceil(size(scan.Ranges,1)/2));
    scan = left_scan.LatestMessage;
    left = scan.Ranges(ceil(size(scan.Ranges,1)/2));
    scan = right_scan.LatestMessage;
    right = scan.Ranges(ceil(size(scan.Ranges,1)/2));
    
    if state == 0
        if fwd > TH_F && left < TH_D && right < TH_D
            vel.Linear.X = 1;
        else
            vel.Linear.X = 0;
            state = 1;
        end
    end
    if state == 1
        if fwd <= TH_F
                vel.Linear.X = 0;
                state = 2;
        else
            if left > TH_D || right > TH_D
                find_middle(velcmd, vel, left_scan, right_scan);
                state = 2;
            end
        end
    end
    if state == 2
        % 2 - right | 1 - left | 0 - fwd
        dir = 0;
        if left >= TH_D || right >= TH_D
            if left >= TH_D && right >= TH_D
                dir = randi([0 1], 1);
            else
                if left >= TH_D
                    dir = 1;
                else
                    dir = 2;
                end
            end
        end
        if dir == 0
            turn(1, velcmd, vel, odom_scan);
            turn(1, velcmd, vel, odom_scan);
            state = 0;
        else
            state = 3;
        end
    end
    if state == 3
        turn(dir, velcmd, vel, odom_scan);
        nudge(velcmd, vel);
        state = 0;
    end
    if state == 4
        vel.Linear.X = 0;
        vel.Angular.Z = 0;
    end
   
    
    send(velcmd, vel);
    pause(0.1);
end
%%
[resetclient, resetmsg] = rossvcclient('/reset_positions');
resetclient.call(resetmsg);
%%
find_middle(velcmd, vel);
%%
turn(1, velcmd, vel, odom_scan);
%%
function a = turn(dir, velcmd, vel, odom_scan)
    turn_transition = [-1, -0.7071; -0.7071, 0; 0, 0.7071; 0.7071, -1; 1, -0.7071];
    angle = odom_scan.LatestMessage.Pose.Pose.Orientation.Z;
    
    DELTA = 0.01;
    
    [val, idx]=min(abs(turn_transition(:,dir)-angle));
    if dir == 1
        target = turn_transition(idx,2);
    else
        target = turn_transition(idx,1);
    end
    
    if dir == 1
        vel.Angular.Z =  0.2;
    else
        vel.Angular.Z =  -0.2;
    end
    
    vel.Linear.X = 0;
    
    while abs(angle-target) > DELTA
        % keep last and then reverse way of turning
        send(velcmd, vel);
        pause(0.5);
        angle = odom_scan.LatestMessage.Pose.Pose.Orientation.Z;
    end
    
    vel.Angular.Z = 0;
    send(velcmd, vel);
end

function b = find_middle(velcmd, vel, left_scan, right_scan)
    left = left_scan.LatestMessage.Ranges;
    right = right_scan.LatestMessage.Ranges;
    
    if right > left
        target = right;
        scan = right_scan;
    else
        scan = left_scan;
        target = left;
    end

    vel.Linear.X = 0.1;
    vel.Angular.Z = 0;
    while target >= 0.33
        send(velcmd, vel);
        pause(0.1);
        target = scan.LatestMessage.Ranges;
    end
end

function c = nudge(velcmd, vel)
    vel.Linear.X = 0.5;
    vel.Angular.Z = 0;
    send(velcmd, vel);
    pause(0.3);
end