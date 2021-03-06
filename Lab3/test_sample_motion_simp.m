clear;

% how many samples do you want to generate
num_samp = 1000;

% initialise test motor command
% x,y, theta and previous, current
u=zeros(3,2,num_samp);

% initialise pose estimate samples 
x=zeros(3,1,num_samp);

% row = (x,y,theta), column = (recent, previous)
% move forward by 1(m) in each time step along x axis
u(3,1,:)=1;	

% initial state estimate (use same for each sample)
x_1 = [0 0 0];

% generate samples of transistion probability distribution
for i=1:num_samp
    x(:,i) = sample_motion_model_simp(u(:,:,i),x_1);
end


% display the results
figure(1)
clf;

plot(x(1,:),x(2,:),'.');
hold on
plot(x_1(1),x_1(2),'ro');
axis([-4 4 -4 4])
