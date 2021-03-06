rosinit
%%
fwd_scan = rossubscriber('base_scan_0');
left_scan = rossubscriber('base_scan_1');
right_scan = rossubscriber('base_scan_2');
odom = rossubscriber('odom');
[velcmd, vel] = rospublisher('/cmd_vel');

last_pos = [0, 0, 0];
last = 0;
while(1)
    if last_pos(1) == odom.LatestMessage.Pose.Pose.Position.X && last_pos(2) == odom.LatestMessage.Pose.Pose.Position.Y && last_pos(3) == odom.LatestMessage.Pose.Pose.Orientation.W
        [resetclient, resetmsg] = rossvcclient('/reset_positions');
        resetclient.call(resetmsg);
    end
    
    last_pos(1) = odom.LatestMessage.Pose.Pose.Position.X;
    last_pos(2) = odom.LatestMessage.Pose.Pose.Position.Y;
    last_pos(3) = odom.LatestMessage.Pose.Pose.Orientation.W;
    
    fwd = fwd_scan.LatestMessage.Ranges;
    left = left_scan.LatestMessage.Ranges;
    right = right_scan.LatestMessage.Ranges;
    
    delta = left - right;
    if (right < 2 && left < 2 && delta <= 0.2)
        vel.Angular.Z = delta*0.85+(delta-last)*5;
        last = delta;
    end
    
    if fwd > 0.4
        vel.Linear.X = 1;
    else
        vel.Linear.X = 0;
        if left > right
            vel.Angular.Z = 2;
        else
            vel.Angular.Z = -2;
        end
    end
    
    send(velcmd, vel);
    pause(0.1);
end
%%
[resetclient, resetmsg] = rossvcclient('/reset_positions')
resetclient.call(resetmsg)