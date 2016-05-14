function plot_rpy(acc_rots, gyro_rots, vicon_rots, ts_imu, ts_vicon,legend1,legend2,legend3)
% By Yiren Lu at University of Pennsylvania
% Feb 10 2016
% ESE 650 Project 2

% This function plot the roll pitch yaw values of rotation matrices
% from acc, gyro and vicon



[r,p,y]=rot2rpy(vicon_rots);
[r2,p2,y2]=rot2rpy(acc_rots);
[r3,p3,y3]=rot2rpy(gyro_rots);
maxr = 0;
maxp = 0;
maxy = 0;
maxrid = 0;
maxpid = 0;
maxyid = 0;

for i = 1:size(ts_vicon,2)
    idx = max(sum((ts_imu < ts_vicon(i))), 1);
    ri = r(i);
    pi = p(i);
    yi = y(i);
    r_3i = r3(idx);
    p_3i = p3(idx);
    y_3i = y3(idx);
    if(maxr<abs(ri - r_3i))
        maxr= abs(ri - r_3i);
        maxrid = idx;
    end
    if(maxp<abs(pi - p_3i))
        maxp= abs(pi - p_3i);
        maxpid = idx;
    end
    if(maxy<abs(yi - y_3i))
        maxy= abs(yi - y_3i);
        maxyid = idx;
    end
end
figure;
subplot(3,1,1), plot(ts_imu,r2,'g-', ts_imu, r3,'r-', ts_vicon,r,'b-', ts_imu(maxrid), r3(maxrid), 'r*');
ylabel('roll');
legend(legend1,legend2, legend3);
title(sprintf('max diffr: %0.2f maxid = %d', maxr,maxrid))

subplot(3,1,2), plot(ts_imu,p2,'g-', ts_imu, p3,'r-', ts_vicon,p,'b-', ts_imu(maxpid), p3(maxpid), 'r*');
ylabel('pitch');
legend(legend1,legend2, legend3);
title(sprintf('max diffp: %0.2f maxid = %d', maxp,maxpid))

subplot(3,1,3), plot(ts_imu,y2,'g-', ts_imu, y3,'r-', ts_vicon,y,'b-',ts_imu(maxyid), y3(maxyid), 'r*');
ylabel('yaw');
legend(legend1,legend2, legend3);
title(sprintf('max diffy: %0.2f maxid = %d', maxy, maxyid))