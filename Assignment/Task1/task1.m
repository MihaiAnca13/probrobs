rosinit
%%
fwd_scan = rossubscriber('base_scan_0');
left_scan = rossubscriber('base_scan_1');
right_scan = rossubscriber('base_scan_2');
[velcmd, vel] = rospublisher('/cmd_vel');

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
    
    disp(left);
    disp(right);
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
                find_middle(velcmd, vel);
                state = 2;
            end
        end
    end
    if state == 2
        % 0 - right | 1 - left
        dir = 2;
        if left >= TH_D || right >= TH_D
            if left >= TH_D && right >= TH_D
                dir = randi([0 1], 1);
            else
                if left >= TH_D
                    dir = 1;
                else
                    dir = 0;
                end
            end
        end
        if dir == 2
            turn(0, velcmd, vel);
            turn(0, velcmd, vel);
            state = 0;
        else
            state = 3;
        end
    end
    if state == 3
        turn(dir, velcmd, vel);
        find_middle(velcmd, vel);
        state = 0;
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
turn(0, velcmd, vel);
%%
function a = turn(dir, velcmd, vel)
    if dir == 0
        vel.Angular.Z =  -3;
    else
        vel.Angular.Z =  3;
    end
    
    vel.Linear.X = 0;
    
    for i = 1:10
        send(velcmd, vel);
        pause(0.1);
    end
    
    vel.Angular.Z = 0;
    send(velcmd, vel);
end

function b = find_middle(velcmd, vel)
    vel.Linear.X = 0.9;
    vel.Angular.Z = 0;
    for i = 1:3
        send(velcmd, vel);
        pause(0.1);
    end
end