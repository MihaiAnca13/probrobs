function ModelCallback(~, message)
    global robot_pose;
    global estimate_x;
    global estimate_y;
    global estimate_pose;
    global vel;
    global velcmd;

    p = robot_pose;
    c = 30;
    send(velcmd, vel);
    tic;
    while c > 0
        last_p = p;
        p = robot_pose;
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
    
    estimate_pose = vel_model(vel, estimate_pose, delta);
    estimate_x = [estimate_x estimate_pose(1)];
    estimate_y = [estimate_y estimate_pose(2)];
end