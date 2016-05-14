function qout = quatdivide( q, r )
%  QUATDIVIDE Divide a quaternion by another quaternion.
%   N = QUATDIVIDE( Q, R ) calculates the result of quaternion division, N,
%   for two given quaternions, Q and R.  Inputs Q and R can be either M-by-4
%   matrices containing M quaternions, or a single 1-by-4 quaternion.
%   N returns an M-by-4 matrix of quaternion quotients.  Each element of Q
%   and R must be a real number.  Additionally, Q and R have their scalar
%   number as the first column.
%
%   Examples:
%
%   Determine the division of two 1-by-4 quaternions:
%      q = [1 0 1 0];
%      r = [1 0.5 0.5 0.75];
%      d = quatdivide(q, r)
%
%   Determine the division of 2-by-4 by a 1-by-4 quaternions:
%      q = [1 0 1 0; 2 1 0.1 0.1];
%      r = [1 0.5 0.5 0.75];
%      d = quatdivide(q, r)
%
%   See also QUATCONJ, QUATINV, QUATMOD, QUATMULTIPLY, QUATNORM,
%   QUATNORMALIZE, QUATROTATE.

%   Copyright 2000-2006 The MathWorks, Inc.

qout = quatmultiply(quatinv( r ), q);