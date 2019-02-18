function pose = vel_model(vel, last_pose, delta_t)
    v = vel.Linear.X;
    w = vel.Angular.Z;
    
    if w == 0
        pose(1) = last_pose(1) + v * delta_t * cos(last_pose(3));
        pose(2) = last_pose(2) + v * delta_t * sin(last_pose(3));
        pose(3) = last_pose(3);
    else
        pose(1) = last_pose(1) + -v/w*sin(last_pose(3)) + v/w*sin(last_pose(3)+w*delta_t);
        pose(2) = last_pose(2) + v/w*cos(last_pose(3)) - v/w*cos(last_pose(3)+w*delta_t);
        pose(3) = last_pose(3) + w*delta_t;
    end
end