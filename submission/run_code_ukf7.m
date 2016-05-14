% By Yiren Lu at University of Pennsylvania
% Feb 19 2016
% ESE 650 Project 2

% This is the main run script, TA starts here.
% 
% Updated Feb 19: I updated the Unscented Kalman Filter. Now it works
% proporly. 
% 
% Youtube video: https://youtu.be/c10qWCd7E4U

%% Cleaning
clear all
addpath ./util
addpath ./quat
addpath ./cam


% Load and preprocess data
% Please update the paths below, execute 1 dataset each time.
% Please put the testsets in the corresponding folders as dataset 10.

dataset = 10;
load(sprintf('vicon/viconRot%d.mat', dataset));
% load('vicon/viconRotTest.mat');
ts_vicon = ts;
rots_vicon = rots;
% rots_vicon = rots(:,:,200:end);
% load('imu/imuRawTest.mat');
load(sprintf('imu/imuRaw%d.mat', dataset));
ts_imu = ts;


acc = vals(1:3,:);      % raw data from accelerometer
gyro = vals(4:6, :);    % raw data from gyroscope

% data preprocessing: convert the acc and gyro data to desirable format and generate rotation matrix.
[acc_vals rots_acc] = acc_data(acc);
[gyro_vals rots_gyro qdelta] = gyro_data(gyro, ts_imu);

plot_rpy(rots_acc,rots_gyro, rots_vicon,ts_imu, ts_vicon, 'acc', 'gyro', 'vicon');
print('-dpng','-r288',sprintf('results/naive_approach_%d.png', dataset));

%% ukf

% P = 10e-3* diag([ones(3,1);ones(3,1)]);   % State covariance matrix
% % Q = 3e-6* diag([ones(3,1);ones(3,1)]);   % Process noise covariance matrix
% Q = 0.07e-6* diag([ones(3,1);ones(3,1)]);   % Process noise covariance matrix
% % good para for dataset3
% % R = diag([2.8e-3*ones(3,1);10e-4*ones(3,1)]);   % Measurement noise covariance matrix
% R = diag([2.8e-4*ones(3,1);10e-4*ones(3,1)]);   % Measurement noise covariance matrix

P = diag([ones(3,1);ones(3,1)]);   % State covariance matrix
Q = 5e-8*[ones(3,3)+eye(3), zeros(3,3);zeros(3,3), ones(3,3)+eye(3)];   % Process noise covariance matrix
R = [2.8e-4*(ones(3,3)+eye(3)), zeros(3,3);zeros(3,3),10e-4*(ones(3,3)+eye(3))];   % Measurement noise covariance matrix

start =1;
[q, rotsUKF] = ukf_7(acc_vals(:,start:end), gyro_vals(:,start:end), qdelta, P, Q, R, ts_imu(start:end));

% visualize UKF vs Vicon
plot_rpy(rots_acc, rotsUKF, rots_vicon,ts_imu, ts_vicon, 'acc', 'UKF', 'vicon');
print('-dpng','-r288',sprintf('results/UKF_%d.png', dataset));



% image stitching

% clear all;


% load(sprintf('results/fuse_rot%d.mat', dataset));
rots = rotsUKF; %rots_fuse;
% rots = rots_raw_fuse;
t   = ts;
load(sprintf('cam/cam%d.mat', dataset));
% load('camTest.mat');
t_cam   = ts;


r = 230;
[h,w,~,~] = size(cam);
screen_height = 600;
screen_width = ceil(2*pi*r)+2;
offset_x = screen_width/2;
offset_y = screen_height/2;
canvas = zeros(screen_height, screen_width, 3, 'uint8');
figure;
for i = 300:2:length(cam) - 250
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
