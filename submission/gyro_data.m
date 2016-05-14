function [gyro_vals gyro_rots qd] = gyro_data(gyro, ts_imu)
% By Yiren Lu at University of Pennsylvania
% Feb 10 2016
% ESE 650 Project 2

% This function parse and convert the raw gyroscope data to desirable
% format and generate rotation matrices (DCM) and q detla


% compute bias & scale
bias        = mean(gyro(:, 1 : 150), 2);
Vref        = 3300;
sensitivity = 3.3;
scale       = Vref/1023/sensitivity*pi/180;

% compute anguler velocities
gyro_vals = scale .* bsxfun(@minus, gyro, bias);
conv_axis = [0 1 0; 0 0 1; 1 0 0];
gyro_vals = conv_axis * gyro_vals;

% compute angles and axis
v_norm = sqrt(sum(gyro_vals.^2, 1));
dt     = ts_imu - [ts_imu(1), ts_imu(1 : end - 1)];
angles   = v_norm.*dt;
axis   = bsxfun(@rdivide, gyro_vals, v_norm); 

% convert to quaternion format
qd = [cos(angles/2)' bsxfun(@times, sin(angles/2), axis)'];
q = [1 0 0 0];
gyro_rots = zeros(3, 3, size(gyro,2));

for i = 1:size(gyro,2)
    q = quatnormalize(quatmultiply(q, qd(i, :)));
	gyro_rots(:,:,i) = quat2dcm(q)';
end
% q= q(200:end);
% qd = qd(200:end,:);
% gyro_rots = gyro_rots(:,:,200:end);
% gyro_vals = gyro_vals(:,200:end);