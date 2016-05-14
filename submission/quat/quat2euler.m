function angles = quat2euler( q )
% QUAT2EULER Obsolete conversion for quaternion to Euler angles.
%   QUAT2EULER is obsolete and may be removed from future versions. Please
%   use QUAT2ANGLE instead.
%
%   See also QUAT2ANGLE.

%  QUAT2EULER Convert quaternion to Euler angles.
%   N = QUAT2EULER( Q ) calculates the Euler angles, N, for a
%   given quaternion, Q.  Input Q is an M-by-4 matrix containing M
%   quaternions.  N returns a M-by-3 matrix of Euler angles.  Each element
%   of Q must be a real number.  Additionally, Q has its scalar number as
%   the first column.  Euler angles are output in radians.
%
%   Examples:
%
%   Determine the Euler angles from q = [1 0 1 0]:
%      ea = quat2euler([1 0 1 0])
%
%   Determine the Euler angles from multiple quaternions:
%      q = [1 0 1 0; 1 0.5 0.3 0.1];
%      ea = quat2euler(q)
%
%   See also ANGLE2DCM, DCM2ANGLE, DCM2QUAT, ANGLE2QUAT, QUAT2DCM.

%   Copyright 2000-2007 The MathWorks, Inc.

warning(message('aero:quat2euler:ObsoleteFunction'));

qin = quatnormalize( q );

phi = atan2(2.*(qin(:,3).*qin(:,4) + qin(:,1).*qin(:,2)), ...
                 qin(:,1).^2 - qin(:,2).^2 - qin(:,3).^2 + qin(:,4).^2);

theta = asin(-2.*(qin(:,2).*qin(:,4) - qin(:,1).*qin(:,3)));

psi = atan2(2.*(qin(:,2).*qin(:,3) + qin(:,1).*qin(:,4)), ...
                 qin(:,1).^2 + qin(:,2).^2 - qin(:,3).^2 - qin(:,4).^2);

angles = [phi theta psi];