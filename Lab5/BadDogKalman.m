%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1D Kalman filter: Bad Dog Kalman! example
% Martin J. Pearson
% Probabilistic Robotics, UWE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
duration   = 10;	% time of simulation (seconds)
dt 	       = 0.1;	% time step of simulation (seconds)
measnoise  = 4; 	% position measurement noise (metres)
accelnoise = 5; 	% acceleration noise (metres/sec^2)
u	       = 1;		% accelerator pedal flat to the floor!


A    = [1 dt; 0 1]; 	% transition matrix
B    = [dt^2/2; dt]; 	% input matrix
C    = [1 0]; 		    % transformation matrix
x    = [0; 0]; 		    % initial state vector [position; velocity]
xhat = x; 		        % initial state estimate
z    = 0;		        % observation [position]

Q = measnoise^2; 					% measurement error covariance
R = accelnoise^2 * [dt^4/4 dt^3/2; dt^3/2 dt^2]; 	% process noise cov
P = R; 							% initial estimation covariance

% Initialize arrays for later plotting.
pos 	= []; % true position 
poshat 	= []; % estimated position 
posmeas = []; % measured position 
vel 	= []; % true velocity 
velhat 	= []; % estimated velocity 
time    = []; % accumulated time for plots


%% Generate noisy motion and measurements
for t = 0 : dt: duration

    % Generate the actual motion of BadDog
    varepsilon = accelnoise * [(dt^2/2)*randn; dt*randn];
    x = A*x + B*u + varepsilon;

    % Generate AngryMan measurements of BadDog
    sigma = measnoise * randn;
    z = C*x + sigma;

    % save state and observations
    pos     = [pos; x(1)];
    posmeas = [posmeas; z];
    vel     = [vel; x(2)];
    time    = [time; t];
end

figure(1)
clf;
plot(time,pos,'-r.')
hold on
plot(time,posmeas,'-k.')
legend('BadDogs actual position', 'measurement of position', 'Location', 'northwest');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Kalman filtering

x    = [0; 0]; 		% re-initialise state
xhat = x; 		% initial state estimate

predict_state = [];
predict_var   = [];

pos_estimate  = [];
vel_estimate  = [];
P_mag_estimate= [];


for t = 1:length(pos)
    % Predict next state of Bad Dog with the last state and predicted motion.
    xhat = A*xhat + B*u;
    predict_state = [predict_state; xhat(1)] ;

    %predict next covariance
    P = A*P*A' + R;
    predict_var = [predict_var; P] ;

    % Kalman Gain
    K = P*C'*inv(C*P*C'+Q);

    % Update the state estimate.
    xhat = xhat + K * (posmeas(t) - C*xhat);

    % update covariance estimation.
    P =  (eye(2)-K*C)*P;

    %Store for plotting
    pos_estimate = [pos_estimate; xhat(1)];
    vel_estimate = [vel_estimate; xhat(2)];
    P_mag_estimate = [P_mag_estimate; P(1)];
end

 figure(2);
 plot(time,pos,'-r.',time,posmeas,'-k.', time,pos_estimate,'-g.');
 legend('BadDogs actual position', 'measurement of position', 'filtered estimate of position', 'Location', 'northwest');

%% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%% plot the evolution of the distributions
% figure(3);
% clf
% x = -5:.01:round(max(posmeas));  % range along x axis
% for T = 1:length(pos_estimate)
% 
%    
%     % predicted next position of BadDog    
%     mu = predict_state(T); 	% mean
%     sigma = predict_var(T); 	% standard deviation
%     y = normpdf(x,mu,sigma); 	% pdf
%     if max(y)>0
%      y = y/(max(y));		
%     end
%     subplot(2,2,[1 2])
%     cla;
%     plot(x,y,'m');
%     hold on %clf
% 
%     subplot(2,2,3)
%     cla;
%     plot(x,y,'m');
%     hold on 	
% 
%     % measurements taken by AngryMan
%     mu = posmeas(T); 		% mean
%     sigma = measnoise; 		% standard deviation
%     y = normpdf(x,mu,sigma); 	% pdf
%     if max(y)>0
%      y = y/(max(y));
%     end
%     
%     subplot(2,2,[1 2])
%     plot(x,y,'k'); 
% 
%     subplot(2,2,3)
%     plot(x,y,'k'); 
%    
%     % combined position estimate
%     mu = pos_estimate(T); 	% mean
%     sigma = P_mag_estimate(T); 	% standard deviation
%     y = normpdf(x,mu,sigma); 	% pdf
%     if max(y)>0
%      y = y/(max(y));
%     end
%     subplot(2,2,[1 2])
%     plot(x,y,'g'); 
%     axis([pos_estimate(T)-1 pos_estimate(T)+1 0 1]);
% 
%     subplot(2,2,3)
%     plot(x,y,'g'); 
%    
% 
%     % actual position of BadDog
%     subplot(2,2,[1 2])
%     line([pos(T);pos(T)],[0;1],'linewidth',2,'color','b');
%     axis([pos(T)-1 pos(T)+1 0 1]);
%     legend('state predicted','measurement','state estimate','BadDogs actual position')
% 
%     subplot(2,2,3)
%     line([pos(T);pos(T)],[0;1],'linewidth',2,'color','b');
%     xlabel('Position');
%     axis([-5 round(max(posmeas))+5 0 1])
%     
%     % plot position against time for reference
%     subplot(2,2,4)
%     plot(time(T),pos(T),'-r.',time(T),posmeas(T),'-k.', time(T),pos_estimate(T),'-g.');
%     hold on
%     axis([0 duration 0 round(max(posmeas))])
%     pause(0.1)
% end
