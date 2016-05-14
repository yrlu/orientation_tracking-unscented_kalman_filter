function qout = quatnorm( q )
%  QUATNORM Calculate the norm of a quaternion.
%   N = QUATNORM( Q ) calculates the norm, N, for a given quaternion, Q.  Input
%   Q is an M-by-4 matrix containing M quaternions.  N returns a column vector
%   of M norms.  Each element of Q must be a real number.  Additionally, Q has
%   its scalar number as the first column.
%
%   Examples:
%
%   Determine the norm of q = [1 0 0 0]:
%      norm = quatnorm([1 0 0 0])
%
%   See also QUATCONJ, QUATDIVIDE, QUATINV, QUATMOD, QUATMULTIPLY,
%   QUATNORMALIZE, QUATROTATE.

%   Copyright 2000-2011 The MathWorks, Inc.

if any(~isreal(q(:)))
    error(message('aero:quatnorm:isNotReal'));
end

if (size(q,2) ~= 4)
    error(message('aero:quatnorm:wrongDimension'));
end

for index = size(q,1):-1:1
    qnorm(index,:) = norm(q(index,:),2);
end

qout = qnorm.*qnorm;