function [rot] = rot2rpy(roll, pitch, yaw)
% By Yiren Lu at Univ. of Pennsylvania, Feb 8, 2016
% ESE 650 Project 2

% Reference: http://planning.cs.uiuc.edu/node102.html



rot = zeros(3,3,size(roll,1));
for i = 1:size(roll,1)
    a = yaw(i);
    b = pitch(i);
    c = roll(i);
    rot(:,:,i) = [cos(a)*cos(b), cos(a).*sin(b).*sin(c)-sin(a).*cos(c), cos(a).*sin(b).*cos(c)+sin(a).*sin(c); 
        sin(a).*cos(b), sin(a).*sin(b).*sin(c)+cos(a).*cos(c), sin(a).*sin(b).*cos(c)-cos(a).*sin(c);
       -sin(b), cos(b).*sin(c), cos(b).*cos(c)];
end

% rot = [cos(a).*cos(b), cos(a).*sin(b).*sin(c)-sin(a).*cos(c), cos(a).*sin(b).*cos(c)+sin(a).*sin(c);
%        sin(a).*cos(b), sin(a).*sin(b).*sin(c)+cos(a).*cos(c), sin(a).*sin(b).*cos(c)-cos(a).*sin(c);
%        -sin(b), cos(b).*sin(c), cos(b).*cos(c)];

% rot = [ cos(yaw).*cos(pitch) - sin(roll).*sin(yaw).*sin(pitch),... 
%       cos(pitch).*sin(yaw) + cos(yaw).*sin(roll).*sin(pitch), -cos(roll).*sin(pitch);...
%       
%       -cos(roll).*sin(yaw),     cos(roll).*cos(yaw),             sin(roll);
%       
%       cos(yaw).*sin(pitch) + cos(pitch).*sin(roll).*sin(yaw),...
%       sin(yaw).*sin(pitch) - cos(yaw).*cos(pitch).*sin(roll),...
%       cos(roll).*cos(pitch)
%     ];
end