%% Cartesian to polar coordinate transform
function [r,theta] = cart2pol(A,B)

r = sqrt((A(1)-B(1))^2+(A(2)-B(2))^2);  % magnitude = sqrt(dX^2 + dY^2)

theta = atan2((B(2)-A(2)),(B(1)-A(1))); % angle = atan2(Y,X)
