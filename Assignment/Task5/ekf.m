load('data.mat');
%%
global data;
global vel;
miu = [4.5 2.2 3.1416];
sigma = zeros(3,3);
pose_data = zeros(length(data),3);

for i = 1:length(data)
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
    
    
    %overwrite old values
    sigma = sigma_cap;
    miu = miu_cap;
    
    pose_data(i,:) = miu;
end

figure;
plot(pose_data(:,1), pose_data(:,2), 'r');
axis([-5 5 -2.5 2.5]);

%%
function miu = g(miu_1, u)
    miu = [0 0 0];
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