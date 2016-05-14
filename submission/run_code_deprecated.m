% By Yiren Lu at University of Pennsylvania
% Feb 10 2016
% ESE 650 Project 2

% This is the main run script, TA starts here.
% 
% The following code is mainly the comparison among Naive approach,
% UKF, and Fuse.

% UPDATE: Please execute run_code_ukf7.m

%% Cleaning
clear all
addpath ./util
addpath ./quat
addpath ./cam


% Load and preprocess data
% Please update the paths below, execute 1 dataset each time.

dataset = 100;
% load(sprintf('vicon/viconRot%d.mat', dataset));
load('vicon/viconRotTest.mat');
ts_vicon = ts;
rots_vicon = rots;
% rots_vicon = rots(:,:,200:end);
load('imu/imuRawTest.mat');
% load(sprintf('imu/imuRaw%d.mat', dataset));
ts_imu = ts;


acc = vals(1:3,:);      % raw data from accelerometer
gyro = vals(4:6, :);    % raw data from gyroscope

% data preprocessing: convert the acc and gyro data to desirable format and generate rotation matrix.
[acc_vals rots_acc] = acc_data(acc);
[gyro_vals rots_gyro qdelta] = gyro_data(gyro, ts_imu);

% ts_imu = ts_imu(200:end);
% ts_vicon = ts_vicon(200:end);

%% visualization of naive approach roll pitch yaw
% directly convert the accelerometer and gyro data into rotation matrices.
plot_rpy(rots_acc,rots_gyro, rots_vicon,ts_imu, ts_vicon, 'acc', 'gyro', 'vicon');
print('-dpng','-r288',sprintf('results/naive_approach_%d.png', dataset));
%% ukf
% Do 4 state UKF, I have also implemented 7 state UKF (ukf_7.m), in this
% project I found 4 state ukf fused with gyro works better for most of the
% time

ukf4 = 0;

if(ukf4==1)
P = 1* diag([ones(3,1)]);  % State covariance matrix
Q = 0.5* diag([ones(3,1)]);% Process noise covariance matrix
R = 0.5* diag([ones(3,1)]);% Measurement noise covariance matrix
else
% P = 1* diag([ones(3,1);ones(3,1)]);  % State covariance matrix
% Q = 0.5* diag([ones(3,1);ones(3,1)]);% Process noise covariance matrix
% R = 0.5* diag([ones(3,1);ones(3,1)]);% Measurement noise covariance matrix


P = 10e-3* diag([ones(3,1);ones(3,1)]);   % State covariance matrix
% Q = 3e-6* diag([ones(3,1);ones(3,1)]);   % Process noise covariance matrix
Q = 0.03e-6* diag([ones(3,1);ones(3,1)]);   % Process noise covariance matrix
% good para for dataset3
% R = diag([2.8e-3*ones(3,1);10e-4*ones(3,1)]);   % Measurement noise covariance matrix
R = diag([2.8e-4*ones(3,1);10e-4*ones(3,1)]);   % Measurement noise covariance matrix

end 

start =1;
if(ukf4==1)
[q, rotsUKF] = ukf_4(acc_vals(:,start:end), gyro_vals(:,start:end), qdelta, P, Q, R, ts_imu(start:end));
else
[q, rotsUKF] = ukf_7(acc_vals(:,start:end), gyro_vals(:,start:end), qdelta, P, Q, R, ts_imu(start:end));
end

% visualize UKF vs Vicon
plot_rpy(rots_acc, rotsUKF, rots_vicon,ts_imu, ts_vicon, 'acc', 'UKF', 'vicon');
print('-dpng','-r288',sprintf('results/UKF_%d.png', dataset));
%% fuse UKF & Gyro
% for some dataset the fusion of UKF outputs(roll, pitch) with gyro output
% (yaw) yields better results
[r_ukf,p_ukf,y_ukf]=rot2rpy(rotsUKF);
[r_gyro,p_gyro,y_gyro]=rot2rpy(rots_gyro);
rots_fuse = rpy2rot(r_ukf,p_ukf,y_gyro);

plot_rpy(rots_acc, rots_fuse, rots_vicon,ts_imu, ts_vicon, 'acc', 'fuse', 'vicon');
print('-dpng','-r288',sprintf('results/Fuse_%d.png', dataset));

%% fuse IMU & Gyro
[r_acc,p_acc,y_acc]=rot2rpy(rots_acc);
[r_gyro,p_gyro,y_gyro]=rot2rpy(rots_gyro);
rots_raw_fuse = rpy2rot(r_acc,p_acc,y_gyro);


%% visualization of rotating boxes
for i = 1:(min(size(rots_vicon,3)/50, size(rots_acc,3)/50) -1)
    subplot(1,4,1), rotplot(rots_acc(:,:,i*50));
    subplot(1,4,2), rotplot(rots_gyro(:,:,i*50));
    subplot(1,4,3), rotplot(rots_fuse(:,:,i*50));
    subplot(1,4,4), rotplot(rots_vicon(:,:,i*50));
    pause(0.01);
end


%% image stitching

% clear all;

dataset = 8;
% load(sprintf('results/fuse_rot%d.mat', dataset));
rots = rotsUKF; %rots_fuse;
% rots = rots_raw_fuse;
t   = ts;
% load(sprintf('cam/cam%d.mat', dataset));
load('camTest.mat');
t_cam   = ts;


r = 230;
[h,w,~,~] = size(cam);
screen_height = 600;
screen_width = ceil(2*pi*r)+2;
offset_x = screen_width/2;
offset_y = screen_height/2;
canvas = zeros(screen_height, screen_width, 3, 'uint8');

for i = 300:5:length(cam) - 200
    i
    img = cam(:,:,:,i);
    img_i = find(t > t_cam(i), 1, 'first');
    dcm = rots(:,:,img_i);
    [x_img, y_img] = meshgrid(1:w, 1:h);
    x_img = x_img(:); 
    y_img = y_img(:); 
    z_img = ones(size(y_img)) * r;
    pts_img = bsxfun(@plus, [z_img'; -x_img'; -y_img'], [0; w/2; h/2]);
    pts_img_rot = dcm * pts_img;
    a = atan2(pts_img_rot(2,:), pts_img_rot(1,:));
    b = atan2(pts_img_rot(3,:), sqrt(pts_img_rot(1,:).^2 + pts_img_rot(2,:).^2));
%     b = bsxfun(@rdivide, pts_img_rot(3,:), sqrt(pts_img_rot(1,:).^2 + pts_img_rot(2,:).^2));
    proj_x   = round(-r * a + offset_x);
    proj_y   = round(-r * b + offset_y);
    
    for k = 1:length(proj_x)
        if proj_y(k) < screen_height - 1 && proj_y(k) > 1 
            canvas(proj_y(k), proj_x(k), :) = img(y_img(k), x_img(k), :);
        end
    end
    imshow(canvas)
    drawnow
end
print('-dpng','-r288',sprintf('results/Fuse_image_stitching_%d.png', dataset));
