% Sample from a Gaussian (p.124 Thrun)
function [x] = sample_norm(b)

b=sqrt(b);

x = 0.5* sum((rand(1,12)*(b*2))-b);