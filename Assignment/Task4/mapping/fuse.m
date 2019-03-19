function [new_value] = fuse(old_value, l_free, l_prior)

% convert to log odds form and perform binary Bayes filter update on occupancy probability of cell
new_value = log(old_value/(1-old_value)) + l_free + l_prior;

% convert from log-odds to probability
new_value = 1 - (1/(1+exp(new_value)));
