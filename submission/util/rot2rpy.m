function [roll, pitch, yaw] = rot2rpy(rot)
% By Yiren Lu at Univ. of Pennsylvania, Feb 8, 2016
% ESE 650 Project 2
%
% Reference to http://planning.cs.uiuc.edu/node103.html

% rot = [cos(a).*cos(b), cos(a).*sin(b).*sin(c)-sin(a).*cos(c), cos(a).*sin(b).*cos(c)+sin(a).*sin(c);
%        sin(a).*cos(b), sin(a).*sin(b).*sin(c)+cos(a).*cos(c), sin(a).*sin(b).*cos(c)-cos(a).*sin(c);
%        -sin(b), cos(b).*sin(c), cos(b).*cos(c)];

% b = -asin(rot(3,1,:));
% c = asin(rot(3,2,:)./cos(b));
% a = rot(1,1,:)./cos(b);


yaw = atan2(rot(2,1,:),rot(1,1,:));
pitch = atan2(-rot(3,1,:),sqrt(rot(3,2,:).^2+rot(3,3,:).^2));
roll = atan2(rot(3,2,:),rot(3,3,:));



% roll  = asin(rot(2,3,:));
% yaw   = atan2(-rot(2,1,:)./cos(roll),rot(2,2,:)./cos(roll));
% pitch = atan2(-rot(1,3,:)./cos(roll),rot(3,3,:)./cos(roll));

roll = reshape(roll, [size(roll,3),1]);
yaw = reshape(yaw, [size(yaw,3),1]);
pitch = reshape(pitch, [size(pitch,3),1]);
end