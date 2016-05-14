function qinv = quatinv( q )
%  QUATINV Calculate the inverse of a quaternion.
%   N = QUATINV( Q ) calculates the inverse, N, for a given quaternion, Q.
%   Input Q is an M-by-4 matrix containing M quaternions.  N returns an
%   M-by-4 matrix of inverses.  Each element of Q must be a real number.
%   Additionally, Q has its scalar number as the first column.
%
%   Examples:
%
%   Determine the inverse of q = [1 0 1 0]:
%      qinv = quatinv([1 0 1 0])
%
%   See also QUATCONJ, QUATDIVIDE, QUATMOD, QUATMULTIPLY, QUATNORM,
%   QUATNORMALIZE, QUATROTATE.

%   Copyright 2000-2005 The MathWorks, Inc.

qinv  = quatconj( q )./(quatnorm( q )*ones(1,4));