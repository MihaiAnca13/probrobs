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
pose_data = zeros(length(data)-1,2);

NR_PARTICLES = 500;
particles = repmat([4.5 2.2 3.1416], NR_PARTICLES, 1);

PLOT = 1;

I = imread('warehouse.png'); 

if PLOT
    figure(1);
end

for i = 1:length(data)-1
    if PLOT
        plot(landmarks(:,1), landmarks(:,2), 'go');
        axis([-5 5 -2.5 2.5]);
        hold on;
        h = image([-5 5],[-2.5 2.5],I); 
        uistack(h,'bottom')
        plot(odom_data(i,1), odom_data(i,2), 'ko','linewidth', 8)
    end
    
    vel.Linear.X = data(i,1);
    vel.Angular.Z = data(i,2);
    
    %reinit weigths for each particle
    wt = ones(NR_PARTICLES, 1);
    for p = 1:NR_PARTICLES
        % get velocity model prediction
        particles(p,:) = vel_model(vel, particles(p,:));
        
        % for each observed feature by the robot
        observed_features = data_features(2*i-1:2*i,:);
        for j = 1:length(observed_features)
            zt = [observed_features(:,j);0];
            if sum(zt) > 0
                wt(p) = wt(p) * landmark_model_known_correspondence(zt, j, particles(p,:), landmarks);
            end
        end
    end
    
    if sum(wt) ~= NR_PARTICLES
        selected = rouletteWheel(wt);
        particles = particles(selected, :);
    end

    if PLOT
        plot(particles(:,1),particles(:,2),'r.');
        plot(mean(particles(:,1)), mean(particles(:,2)), 'bo','linewidth', 4)
        pause(0.01);
        hold off;
    end
    pose_data(i,:) = [mean(particles(:,1)) mean(particles(:,2))];
end

if PLOT
%%
    rme_vel = sqrt((estimate_data(:,1) - odom_data(:,1)).^2 + (estimate_data(:,2) - odom_data(:,2)).^2);
    rme_ekf = sqrt((pose_data(:,1) - odom_data(:,1)).^2 + (pose_data(:,2) - odom_data(:,2)).^2);
    dl = size(rme_vel,1);

    figure(2);
    subplot(2,1,1);
    plot(1:dl, rme_vel, 'r');
    subplot(2,1,2);
    plot(1:dl, rme_ekf, 'b');
    
    figure(3);
    subplot(3,1,1);
    plot(pose_data(:,1), pose_data(:,2), 'r');
    hold on;
    plot(landmarks(:,1), landmarks(:,2), 'o');
    hold off;
    axis([-5 5 -2.5 2.5]);
    subplot(3,1,2);
    plot(odom_data(:,1), odom_data(:,2), 'r');
    hold on;
    plot(landmarks(:,1), landmarks(:,2), 'o');
    hold off;
    axis([-5 5 -2.5 2.5]);
    subplot(3,1,3);
    plot(estimate_data(:,1), estimate_data(:,2), 'r');
    hold on;
    plot(landmarks(:,1), landmarks(:,2), 'o');
    hold off;
    axis([-5 5 -2.5 2.5]);
end
%%
function q = landmark_model_known_correspondence(feature, j, x, m)
    r_hat = sqrt((m(j,1)-x(1))^2 + (m(j,2)-x(2))^2);
    theta_hat = wrapToPi(atan2(m(j,2)-x(2), m(j,1)-x(1))-x(3));
    sigma = 0.1;
    q = gaussmf(feature(1)-r_hat, [sigma 0])*gaussmf(feature(2)-theta_hat, [sigma 0]+0.05);
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