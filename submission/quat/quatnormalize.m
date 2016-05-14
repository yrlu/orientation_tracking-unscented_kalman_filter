function qout = quatnormalize( q )
%  QUATNORMALIZE Normalize a quaternion.
%   N = QUATNORMALIZE( Q ) calculates the normalized quaternion, N, for a
%   given quaternion, Q.  Input Q is an M-by-4 matrix containing M
%   quaternions.  N returns an M-by-4 matrix of normalized quaternions.
%   Each element of Q must be a real number.  Additionally, Q has its
%   scalar number as the first column.
%
%   Examples:
%
%   Normalize q = [1 0 1 0]:
%      normal = quatnormalize([1 0 1 0])
%
%   See also QUATCONJ, QUATDIVIDE, QUATINV, QUATMOD, QUATMULTIPLY,
%   QUATNORM, QUATROTATE.

%   Copyright 2000-2005 The MathWorks, Inc.

qout = q./(quatmod( q )* ones(1,4));