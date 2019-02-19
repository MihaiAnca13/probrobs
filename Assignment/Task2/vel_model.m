function pose = vel_model(vel, last_pose, delta_t)
    v = vel.Linear.X;
    w = vel.Angular.Z;
    
    a1=0.1;
    a2=0.02;
    a3=0.2; 
    a4=0.01;
    a5=0.001;
    a6=0.01;
    
    v_cap = v + sample_norm(a1)*v^2 + sample_norm(a2)*w^2;
    w_cap = w + sample_norm(a3)*v^2 + sample_norm(a4)*w^2;
    
    if w == 0
        pose(1) = last_pose(1) + v_cap * delta_t * cos(last_pose(3));
        pose(2) = last_pose(2) + v_cap * delta_t * sin(last_pose(3));
        pose(3) = last_pose(3);
    else
        pose(1) = last_pose(1) + -v_cap/w_cap*sin(last_pose(3)) + v_cap/w_cap*sin(last_pose(3)+w_cap*delta_t);
        pose(2) = last_pose(2) + v_cap/w_cap*cos(last_pose(3)) - v_cap/w_cap*cos(last_pose(3)+w_cap*delta_t);
        pose(3) = last_pose(3) + (w_cap + sample_norm(a5)*v^2 + sample_norm(a6)*w^2)*delta_t;
    end
end