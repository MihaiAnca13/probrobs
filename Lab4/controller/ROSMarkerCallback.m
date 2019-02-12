function ROSMarkerCallback(~, message)
    global GL_markers;
    global GL_NumMarkers;
    global GL_NoisyMarkers;
    global GL_Features;
    global GL_robot_pose;
    
    variance = 0.001;
    
    GL_markers = zeros(2,GL_NumMarkers);
    GL_NoisyMarkers = zeros(2,GL_NumMarkers);
    GL_Features = zeros(2,GL_NumMarkers);
    num = length(message.Markers);
    while num>0
        %% Return Cartesian coordinates of Fiducials in robot frame
        x = message.Markers(num).Pose.Position.X;
        y = message.Markers(num).Pose.Position.Y;
        
        GL_markers(1, message.Markers(num).Ids) = x;
        GL_markers(2, message.Markers(num).Ids) = y;
        
        x = x + sample_norm(variance);
        y = y + sample_norm(variance);
        
        GL_NoisyMarkers(1, message.Markers(num).Ids) = x;
        GL_NoisyMarkers(2, message.Markers(num).Ids) = y;
        
        GL_Features(1, message.Markers(num).Ids) = atan2(y, x);
        GL_Features(2, message.Markers(num).Ids) = sqrt(x^2 + y^2);
        
        num=num-1;
    end
    
    GL_markers;
    GL_NoisyMarkers;
    GL_Features;
end
