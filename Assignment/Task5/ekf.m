load('data.mat');
load('data_features.mat');
load('odom_data.mat');
load('estimate_data.mat');
%%
global data;
global vel;
global data_features;
landmarks = [2.567 2.007; 0.652 2; 3.278 0.348; -1.286 2.003; -3.2 2.003; -4.157 -2.004; -2.206 -2.004; -0.333 -2.004; 1.597 -2.004];
miu = [4.5; 2.2; 3.1416];
sigma = zeros(3,3);
pose_data = zeros(length(data)-1,3);

a5 = 0.01;
a6 = 0.01;
Qt = [a5 0 0; 0 a6 0; 0 0 0.01];
% Qt = Qt.^2;

for i = 1:length(data)-1
    vel.Linear.X = data(i,1);
    vel.Angular.Z = data(i,2);
   
    if vel.Angular.Z == 0
        vel.Angular.Z = vel.Angular.Z + sample_norm(0.00001);
    end
    
    %prediction step
    miu_cap = g(miu, vel);
    Gt = calculate_Gt(miu, vel);
    Rt = calculate_Rt(miu, vel);
    sigma_cap = Gt*sigma*Gt'+Rt;
    
    %update step
    observed_features = data_features(2*i-1:2*i,:);
    for j = 1:length(observed_features)
        zt = [observed_features(:,j);0];
        if sum(zt) > 0
            q = (landmarks(j,1)-miu_cap(1))^2+(landmarks(j,2)-miu_cap(2))^2;
            zt_cap = [sqrt(q); atan2(landmarks(j,2)-miu_cap(2), landmarks(j,1)-miu_cap(1))-miu_cap(3);0];
            zt_cap(2) = wrapToPi(zt_cap(2));
            Ht = calculate_Ht(miu_cap, landmarks(j,:), q);
            Kt = sigma_cap*Ht'* pinv(Ht*sigma_cap*Ht'+Qt);
            miu_cap = miu_cap + (Kt*(zt-zt_cap));
            sigma_cap = (eye(3)-Kt*Ht)*sigma_cap;
        end
    end
    
    %overwrite old values
    sigma = sigma_cap;
    miu = miu_cap;
    
    pose_data(i,:) = miu;
end

% plot trajectories
figure(1);
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

% fix pose_data
% pose_data = [[4.5 2.2 3.1416]; pose_data];
% pose_data = pose_data(1:end-2,:);

rme_vel = sqrt((estimate_data(:,1) - odom_data(:,1)).^2 + (estimate_data(:,2) - odom_data(:,2)).^2);
rme_ekf = sqrt((pose_data(:,1) - odom_data(:,1)).^2 + (pose_data(:,2) - odom_data(:,2)).^2);
dl = size(rme_vel,1);

figure(2);
subplot(2,1,1);
plot(1:dl, rme_vel, 'r');
subplot(2,1,2);
plot(1:dl, rme_ekf, 'b');
%%
function miu = g(miu_1, u)
    miu = [0; 0; 0];
    delta_t=0.1;
    
    v = u.Linear.X;
    w = u.Angular.Z;
    
    miu(1) = miu_1(1) - v/w*sin(miu_1(3)) + v/w*sin(miu_1(3)+w*delta_t);
    miu(2) = miu_1(2) + v/w*cos(miu_1(3)) - v/w*cos(miu_1(3)+w*delta_t);
    miu(3) = wrapToPi(miu_1(3)+w*delta_t);
end

function Gt = calculate_Gt(miu_1, u)
    delta_t=0.1;
    v = u.Linear.X;
    w = u.Angular.Z;
    t = miu_1(3);
    
    el1 = v/w*(-cos(t)+cos(t+w*delta_t));
    el2 = v/w*(-sin(t)+sin(t+w*delta_t));
    
    Gt = [1 0 el1; 0 1 el2; 0 0 1];
end

function Rt = calculate_Rt(miu_1, u)
    a1=0.1;
    a2=0.02;
    a3=0.2; 
    a4=0.01;
    v = u.Linear.X;
    w = u.Angular.Z;
    t = miu_1(3);
    delta_t=0.1;
    
    Mt = [a1*v^2+a2*w^2 0; 0 a3*v^2+a4*w^2];
    
    Vt = zeros(3,2);
    Vt(1,1) = (-sin(t)+sin(t+w*delta_t))/w;
    Vt(2,1) = (cos(t)-cos(t+w*delta_t))/w;
    Vt(3,1) = 0;
    Vt(1,2) = v/w^2*(sin(t)-sin(t+w*delta_t))+v/w*cos(t+w*delta_t)*delta_t;
    Vt(2,2) = -v/w^2*(cos(t)-cos(t+w*delta_t))+v/w*sin(t+w*delta_t)*delta_t;
    Vt(3,2) = delta_t;
    
    Rt = Vt*Mt*Vt';
end

% function q = landmark_model_known_correspondence(features, j, miu, m)
%     r_hat = sqrt(m(j,1)-miu(1)^2 + (m(j,2)-miu(2))^2);
%     theta_hat = atan2(m(j,2)-miu(2), m(j,1)-miu(1))-miu(3);
%     sigma = 0.001;
%     q = gaussmf(features(2)-r_hat, [sigma 0])*gaussmf(features(1)-theta_hat, [sigma 0]);
% end

function Ht = calculate_Ht(miu, landmark, q)
    a1 = -(landmark(1)-miu(1))/sqrt(q);
    a2 = -(landmark(2)-miu(2))/sqrt(q);
    a3 = (landmark(2)-miu(2))/q;
    a4 = -(landmark(1)-miu(1))/q;
    
    Ht = [a1 a2 0; a3 a4 -1; 0 0 0];
end