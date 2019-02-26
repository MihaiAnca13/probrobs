function ROSMarkerCallback(~, message)
    global NumMarkers;
    global NoisyMarkers;
    global Features;
    
    variance = 0.001;
    
    NoisyMarkers = zeros(2,NumMarkers);
    Features = zeros(2,NumMarkers);
    num = length(message.Markers);
    while num>0
        % Return Cartesian coordinates of Fiducials in robot frame
        x = message.Markers(num).Pose.Position.X;
        y = message.Markers(num).Pose.Position.Y;
        
        x = x + sample_norm(variance);
        y = y + sample_norm(variance);
        
        NoisyMarkers(1, message.Markers(num).Ids) = x;
        NoisyMarkers(2, message.Markers(num).Ids) = y;
        
        Features(1, message.Markers(num).Ids) = atan2(y, x);
        Features(2, message.Markers(num).Ids) = sqrt(x^2 + y^2);
        
        num=num-1;
    end
    
    NoisyMarkers;
    Features;
end
