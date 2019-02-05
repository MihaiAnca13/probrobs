rosinit
%%
fwd_scan = rossubscriber('base_scan_0');
left_scan = rossubscriber('base_scan_1');
right_scan = rossubscriber('base_scan_2');
[velcmd, vel] = rospublisher('/cmd_vel');

dist2wall = inf;
last = 0;
while(1)
    fwd = fwd_scan.LatestMessage.Ranges;
    left = left_scan.LatestMessage.Ranges;
    right = right_scan.LatestMessage.Ranges;
    
    if (right < 2 && left < 2)
        delta = left - right;
        vel.Angular.Z = delta+(delta-last);
        last = delta;
    end
    
    if fwd > 0.5
        vel.Linear.X = 1;
    else
        vel.Linear.X = 0;
        vel.Angular.Z = -2;
    end
    
    send(velcmd, vel);
    pause(0.1);
end
%%
[resetclient, resetmsg] = rossvcclient('/reset_positions')
resetclient.call(resetmsg)