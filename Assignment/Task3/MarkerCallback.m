function ROSMarkerCallback(~, message)
    % init globals for number of landmarks, calculations with and without
    % noise
    global NumMarkers;
    global NoisyMarkers;
    global Features;
    
    % amount of noise added
    variance = 0.001;
    
    % init placeholder for calculations
    NoisyMarkers = zeros(2,NumMarkers);
    Features = zeros(2,NumMarkers);
    % get number of landmarks observed
    num = length(message.Markers);
    while num>0
        % Return Cartesian coordinates of Fiducials in robot frame
        x = message.Markers(num).Pose.Position.X;
        y = message.Markers(num).Pose.Position.Y;
        
        %add noise to observations
        x = x + sample_norm(variance);
        y = y + sample_norm(variance);
        
        % 
        NoisyMarkers(1, message.Markers(num).Ids) = x;
        NoisyMarkers(2, message.Markers(num).Ids) = y;
        
        Features(1, message.Markers(num).Ids) = atan2(y, x);
        Features(2, message.Markers(num).Ids) = sqrt(x^2 + y^2);
        
        num=num-1;
    end
    
    NoisyMarkers;
    Features;
end
