rosinit
%%
fwd_scan = rossubscriber('base_scan_0');
left_scan = rossubscriber('base_scan_1');
right_scan = rossubscriber('base_scan_2');
odom_scan = rossubscriber('odom');
[velcmd, vel] = rospublisher('/cmd_vel');

pause(0.5);

%scan = fwd_scan.LatestMessage;
%fwd = scan.Ranges(ceil(size(scan.Ranges,1)/2));
%scan = left_scan.LatestMessage;
%left = scan.Ranges(ceil(size(scan.Ranges,1)/2));
%scan = right_scan.LatestMessage;
%right = scan.Ranges(ceil(size(scan.Ranges,1)/2));

go_init(velcmd, vel, odom_scan);

coords_x = [3.03];
coords_y = [2];

cmds = ['S', 'S', 'S', 'S', 'S', 'S', 'W'];
for i = 1:numel(cmds)
    [x, y] = go_dir(cmds(i), velcmd, vel, odom_scan);
    coords_x = [coords_x x];
    coords_y = [coords_y y];
end

plot(coords_x, coords_y, '-o');
axis([-5 5 -2.5 2.5]);

%%
[resetclient, resetmsg] = rossvcclient('/reset_positions');
resetclient.call(resetmsg);
pause(0.1);
%%
go_dir('S', velcmd, vel, odom_scan);
%%
turn('E', velcmd, vel, odom_scan);
%%
function [] = turn(dir, velcmd, vel, odom_scan)
    DELTA = 0.01;
    vel.Linear.X = 0;

    scan = odom_scan.LatestMessage.Pose.Pose.Orientation;
    vel.Angular.Z = 0.1;
    must_b = 0;
    
    if dir == 'N'
        while ~(scan.Z < 0.7 + DELTA && scan.Z > 0.7 - DELTA && scan.W < 0.7 + DELTA && scan.W > 0.7 - DELTA)
            send(velcmd, vel);
            pause(0.1);
            scan = odom_scan.LatestMessage.Pose.Pose.Orientation;
            must_b = 1;
        end
    end
    
    if dir == 'E'
        while ~(scan.Z < 0 + DELTA && scan.Z > 0 - DELTA && scan.W < 1 + DELTA && scan.W > 1 - DELTA)
            send(velcmd, vel);
            pause(0.1);
            scan = odom_scan.LatestMessage.Pose.Pose.Orientation;
            must_b = 1;
        end
    end
    
    if dir == 'S'
        while ~(scan.Z < -0.7 + DELTA && scan.Z > -0.7 - DELTA && scan.W < 0.7 + DELTA && scan.W > 0.7 - DELTA)
            send(velcmd, vel);
            pause(0.1);
            scan = odom_scan.LatestMessage.Pose.Pose.Orientation;
            must_b = 1;
        end
    end
    
    if dir == 'W'
        while ~(abs(scan.Z) < 1 + DELTA && abs(scan.Z) > 1 - DELTA && scan.W < 0 + DELTA && scan.W > 0 - DELTA)
            send(velcmd, vel);
            pause(0.1);
            scan = odom_scan.LatestMessage.Pose.Pose.Orientation;
            must_b = 1;
        end
    end
    
    if must_b == 1
        vel.Angular.Z = -0.01;
        send(velcmd, vel);
        pause(0.1);
    end
    
    vel.Angular.Z = 0;
    send(velcmd, vel);
end

function [x, y] = go_dir(dir, velcmd, vel, odom_scan)
    turn(dir, velcmd, vel, odom_scan);

    vel.Linear.X = 0.1;

    scan = odom_scan.LatestMessage.Pose.Pose.Position;
    curr_x = scan.X;
    curr_y = scan.Y;
    
    if dir == 'N'
        dist = scan.Y - curr_y;
        while dist <= 0.72
            send(velcmd, vel);
            pause(0.1);
            scan = odom_scan.LatestMessage.Pose.Pose.Position;
            dist = scan.Y - curr_y;
        end
    end
    
    if dir == 'E'
        dist = scan.X - curr_x;
        while dist <= 0.95
            send(velcmd, vel);
            pause(0.1);
            scan = odom_scan.LatestMessage.Pose.Pose.Position;
            dist = scan.X - curr_x;
        end
    end
    
    if dir == 'S'
        dist = curr_y - scan.Y;
        while dist <= 0.72
            send(velcmd, vel);
            pause(0.1);
            scan = odom_scan.LatestMessage.Pose.Pose.Position;
            dist = curr_y - scan.Y;
        end
    end
    
    if dir == 'W'
        dist = curr_x - scan.X;
        while dist <= 0.95
            send(velcmd, vel);
            pause(0.1);
            scan = odom_scan.LatestMessage.Pose.Pose.Position;
            dist = curr_x - scan.X;
        end
    end
    
    vel.Linear.X = -0.1;
    send(velcmd, vel);
    pause(0.1);
    
    vel.Linear.X = 0;
    send(velcmd, vel);
    pause(0.1);
    
    x = odom_scan.LatestMessage.Pose.Pose.Position.X;
    y = odom_scan.LatestMessage.Pose.Pose.Position.Y;    
end

function [] = go_init(velcmd, vel, odom_scan)
    vel.Linear.X = 0.3;
    vel.Angular.Z = 0;
    
    scan = odom_scan.LatestMessage.Pose.Pose.Position;
    
    while scan.X >= 3.03
        send(velcmd, vel);
        pause(0.1);
        scan = odom_scan.LatestMessage.Pose.Pose.Position;
    end
    
    vel.Linear.X = -0.3;
    send(velcmd, vel);
    pause(0.1);
end