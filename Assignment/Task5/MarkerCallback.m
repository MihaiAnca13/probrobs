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
    num = length(message.Markers);
    % get number of landmarks observed
    while num>0
        % Return Cartesian coordinates of Fiducials in robot frame
        x = message.Markers(num).Pose.Position.X;
        y = message.Markers(num).Pose.Position.Y;
        
        % add noise to observation
        x_hat = x + sample_norm(variance);
        y_hat = y + sample_norm(variance);
        
        % save noisy position
        NoisyMarkers(1, message.Markers(num).Ids) = x_hat;
        NoisyMarkers(2, message.Markers(num).Ids) = y_hat;
        
        % save feature of observed landmark using noisy position
        Features(1, message.Markers(num).Ids) = sqrt(x^2 + y^2); % range
        Features(2, message.Markers(num).Ids) = atan2(y, x); % bearing
        
        num=num-1;
    end
end
