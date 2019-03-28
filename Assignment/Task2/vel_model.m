% returns predicted pose of robot based on last pose and vel commands sent
function pose = vel_model(vel, last_pose, delta_t)
    % extracting linear vel and angular vel
    v = vel.Linear.X;
    w = vel.Angular.Z;
    
    % given noise params
    a1=0.1;
    a2=0.02;
    a3=0.2; 
    a4=0.01;
    a5=0.001;
    a6=0.01;
    
    % add noise to linear and angular vel based on linear and angular
    % movement
    v_cap = v + sample_norm(a1)*v^2 + sample_norm(a2)*w^2;
    w_cap = w + sample_norm(a3)*v^2 + sample_norm(a4)*w^2;
    
    % separate case when w = 0 because of division by 0
    if w == 0
        % update coordinates and angle
        pose(1) = last_pose(1) + v_cap * delta_t * cos(last_pose(3));
        pose(2) = last_pose(2) + v_cap * delta_t * sin(last_pose(3));
        pose(3) = last_pose(3);
    else
        % update coordinates and angle
        pose(1) = last_pose(1) + -v_cap/w_cap*sin(last_pose(3)) + v_cap/w_cap*sin(last_pose(3)+w_cap*delta_t);
        pose(2) = last_pose(2) + v_cap/w_cap*cos(last_pose(3)) - v_cap/w_cap*cos(last_pose(3)+w_cap*delta_t);
        pose(3) = last_pose(3) + (w_cap + sample_norm(a5)*v^2 + sample_norm(a6)*w^2)*delta_t;
        % maps angle to [-pi pi]
        pose(3) = wrapToPi(pose(3));
    end
end