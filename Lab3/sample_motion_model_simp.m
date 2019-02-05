%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Algorithm sample motion model odometry, P.136 Thrun
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Martin J. Pearson, Probabilistic Robotics, UWE
%%%%%
function [x] =  sample_motion_model_simp(u,x_1)
%%% Arguments:
% u is a 2x3 matrix, column 1 being most recent odometry and 2 being previous odometry
% x_1 is the previous pose estimate [x y theta]
%%% Return:
% x is the current pose estimate [x y theta]

alpha1 = 0.02;    % noise measured in rotation caused by rotation
alpha2 = 0.0;    % noise measured in rotation caused by translation
alpha3 = 0.3;    % noise measured in translation caused by translation
alpha4 = 0.01;    % noise measured in translation caused by rotation

% initialise output vector
x= zeros(3,1);

% find derivatives from odometry motor command
dx = u(1,1)-u(1,2);
dy = u(2,1)-u(2,2);
dtheta = u(3,1)-u(3,2);

% inverse motion model to find noise free relative motion parameters
delta_rot1  = atan2(dy,dx)-u(3,2);
delta_trans = sqrt(power(dx,2) + power(dy,2));
delta_rot2  = dtheta - delta_rot1;

% Add noise to relative motion parameters taken from distribution that
% represents p(x|u,x_1)
delta_rot1_hat  = delta_rot1  - sample_norm((alpha1*delta_rot1^2) + (alpha2*delta_trans^2));
delta_trans_hat = delta_trans - sample_norm((alpha3*delta_trans^2) + (alpha4*delta_rot1^2) + (alpha4*delta_rot2^2));
delta_rot2_hat  = delta_rot2  - sample_norm((alpha1*delta_rot2^2) + (alpha2*delta_trans^2));

% forward model of motion to find sample estimate of pose
x(1) = x_1(1) + delta_trans_hat * cos(x_1(3) + delta_rot1_hat);
x(2) = x_1(2) + delta_trans_hat * sin(x_1(3) + delta_rot1_hat);
x(3) = x_1(3) + delta_rot1_hat + delta_rot2_hat;


% Sample from a Gaussian (p.124 Thrun)
function [x] = sample_norm(b)

b=sqrt(b);

x = 0.5* sum((rand(1,12)*(b*2))-b);
