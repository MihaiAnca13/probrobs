clear;

% how many samples do you want to generate
num_samp = 1000;

% initialise test motor command
u=zeros(3,2,num_samp);

% initialise pose estimate samples 
x=zeros(3,1,num_samp);

% row = (x,y,theta), column = (recent, previous)
% move forward by 1(m) in each time step along x axis
u(3,1,:)=1;	

% initial state estimate (use same for each sample)
x_1 = [0 0 0];

err=zeros(1,num_samp);
% generate samples of transistion probability distribution
for i=1:num_samp
    x(:,i) = sample_motion_model_hidden(u(:,:,i),x_1);
    err(i) = abs(x(1,:,i));
end

% display the results
figure(2)
clf;

plot(x(1,:),x(2,:),'.');
hold on
plot(x_1(1),x_1(2),'ro');
axis([-4 4 -4 4])
