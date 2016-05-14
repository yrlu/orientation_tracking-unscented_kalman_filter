function [acc_vals acc_rots] = acc_data(acc)
% By Yiren Lu at University of Pennsylvania
% Feb 10 2016
% ESE 650 Project 2

% This function parse and convert the raw accelerometer data to desirable
% format and generate rotation matrices (DCM)

Vref = 3.3; % v
sensitivity = 0.330; % mV/g
scale_factor = Vref/(1023*sensitivity);
acc_bias = mean(acc(:, 1 : 150), 2);
% bias(3)     = - 1/ (Vref / 1023 / sensitivity) + bias(3);
% acc_bias(3) = (acc_bias(1)+acc_bias(2))/2;
% acc_bias = [511; 500; 504];
acc_bias(3,:) = acc_bias(3,:) - 1/ (Vref / 1023 / sensitivity);
acc_vals = bsxfun(@minus, acc, acc_bias)*scale_factor;

% generate roll pitch from acc data
r = zeros(size(acc_vals,1),1);
p = zeros(size(acc_vals,1),1);
y = zeros(size(acc_vals,1),1);

for i = 1:size(acc_vals,2)
    v = acc_vals(:,i);
    p(i) = atan2(v(1), sqrt(v(2)^2 + v(3)^2));
    r(i) = -atan2(v(2), sqrt(v(1)^2 + v(3)^2));
    y(i) = 0;
end

% generate dcm
acc_rots = rpy2rot(r,p,y);
% acc_vals = acc_vals(:,200:end);
% acc_rots = acc_rots(:,:,200:end);
end