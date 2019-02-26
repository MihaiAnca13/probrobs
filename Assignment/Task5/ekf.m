load('data.mat');
%%
global data;
global vel;
miu = [4.5 2.2 3.1416];
pose_data = zeros(length(data),3);

for i = 1:length(data)
    vel.Linear.X = data(i,1);
    vel.Angular.Z = data(i,2);
    miu_cap = g(miu, vel);
    pose_data(i,:) = miu_cap;
    Gt = calculate_Gt(miu, vel)
    
    miu = miu_cap;
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
    if w == 0
        w = w + sample_norm(0.00001);
    end
    
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
    
    Gt = [1 0 el1; 0 1 el2; 0 0 1]
end