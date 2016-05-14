function dcm = quat2dcm( q )
%  QUAT2DCM Convert quaternion to direction cosine matrix.
%   N = QUAT2DCM( Q ) calculates the direction cosine matrix, N, for a
%   given quaternion, Q.  Input Q is an M-by-4 matrix containing M
%   quaternions.  N returns a 3-by-3-by-M matrix of direction cosine
%   matrices.  The direction cosine matrix performs the coordinate
%   transformation of a vector in inertial axes to a vector in body axes.
%   Each element of Q must be a real number.  Additionally, Q has its
%   scalar number as the first column.
%
%   Examples:
%
%   Determine the direction cosine matrix from q = [1 0 1 0]:
%      dcm = quat2dcm([1 0 1 0])
%
%   Determine the direction cosine matrices from multiple quaternions:
%      q = [1 0 1 0; 1 0.5 0.3 0.1];
%      dcm = quat2dcm(q)
%
%   See also ANGLE2DCM, DCM2ANGLE, DCM2QUAT, ANGLE2QUAT, QUAT2ANGLE, QUATROTATE.

%   Copyright 2000-2007 The MathWorks, Inc.

qin = quatnormalize( q );

dcm = zeros(3,3,size(qin,1));

dcm(1,1,:) = qin(:,1).^2 + qin(:,2).^2 - qin(:,3).^2 - qin(:,4).^2;
dcm(1,2,:) = 2.*(qin(:,2).*qin(:,3) + qin(:,1).*qin(:,4));
dcm(1,3,:) = 2.*(qin(:,2).*qin(:,4) - qin(:,1).*qin(:,3));
dcm(2,1,:) = 2.*(qin(:,2).*qin(:,3) - qin(:,1).*qin(:,4));
dcm(2,2,:) = qin(:,1).^2 - qin(:,2).^2 + qin(:,3).^2 - qin(:,4).^2;
dcm(2,3,:) = 2.*(qin(:,3).*qin(:,4) + qin(:,1).*qin(:,2));
dcm(3,1,:) = 2.*(qin(:,2).*qin(:,4) + qin(:,1).*qin(:,3));
dcm(3,2,:) = 2.*(qin(:,3).*qin(:,4) - qin(:,1).*qin(:,2));
dcm(3,3,:) = qin(:,1).^2 - qin(:,2).^2 - qin(:,3).^2 + qin(:,4).^2;