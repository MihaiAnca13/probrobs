load('data.mat');
load('data_features.mat');
load('odom_data.mat');
load('estimate_data.mat');
%%
global data;
global vel;
global data_features;

landmarks = [2.567 2.007; 0.652 2; 3.278 0.348; -1.286 2.003; -3.2 2.003; -4.157 -2.004; -2.206 -2.004; -0.333 -2.004; 1.597 -2.004];
sigma = zeros(3,3);
pose_data = zeros(length(data),3);

NR_PARTICLES = 50;
particles = repmat([4.5 2.2 3.1416], NR_PARTICLES, 1);

figure;

for i = 1:length(data)
    plot(landmarks(:,1), landmarks(:,2), 'go');
    axis([-5 5 -2.5 2.5]);
    hold on;
    
    vel.Linear.X = data(i,1);
    vel.Angular.Z = data(i,2);
    
    %reinit weigths for each particle
    wt = ones(NR_PARTICLES, 1);
    for p = 1:NR_PARTICLES
        % get velocity model prediction
        particles(p,:) = vel_model(vel, particles(p,:));
        
        landmarks_seen = 0;
        % for each observed feature by the robot
        observed_features = data_features(2*i-1:2*i,:);
        for j = 1:length(observed_features)
            zt = [observed_features(:,j);0];
            if sum(zt) > 0
                landmarks_seen = 1;
                wt(p) = wt(p) * landmark_model_known_correspondence(zt, j, particles(p,:), landmarks);
            end
        end
        
        if landmarks_seen
            selected = rouletteWheel(wt);
            particles = particles(selected, :);
        end
        
        plot(particles(p,1),particles(p,2),'b.');
    end
    pause(0.01);
    hold off;
%     pose_data(i,:) = x;
end
%%
function q = landmark_model_known_correspondence(feature, j, x, m)
    r_hat = sqrt((m(j,1)-x(1))^2 + (m(j,2)-x(2))^2);
    theta_hat = wrapToPi(atan2(m(j,2)-x(2), m(j,1)-x(1))-x(3));
    sigma = 0.1;
    q = gaussmf(feature(1)-r_hat, [sigma 0])*gaussmf(feature(2)-theta_hat, [sigma 0]);
end

function selected = rouletteWheel(weights)
    selected = zeros('like', weights);

    accumulation = cumsum(weights);
    
    for i = 1 : length(weights)
        p = rand() * accumulation(end);
        chosen_index = -1;
        for index = 1 : length(accumulation)
            if (accumulation(index) > p)
              chosen_index = index;
              break;
            end
        end
        selected(i) = chosen_index;
    end
end