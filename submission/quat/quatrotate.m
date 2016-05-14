function qout = quatrotate( q, r )
%  QUATROTATE Rotate a vector by a quaternion.
%   N = QUATROTATE( Q, R ) calculates the rotated vector, N, for a
%   quaternion, Q, and a vector, R.  Q is either a M-by-4 matrix
%   containing M quaternions or a single 1-by4 quaternion.  R
%   is either a M-by-3 matrix or a single 1-by-3 vector.  N returns an
%   M-by-3 matrix of rotated vectors.  Each element of Q and R must be a
%   real number.  Additionally, Q has its scalar number as the first column.
%
%   Examples:
%
%      q = [1 0 1 0];
%      r = [1 1 1];
%      n = quatrotate( q, r )
%
%      q = [1 0 1 0; 1 0.5 0.3 0.1];
%      r = [1 1 1];
%      n = quatrotate( q, r )
%
%      q = [1 0 1 0];
%      r = [1 1 1; 2 3 4];
%      n = quatrotate( q, r )
%
%      q = [1 0 1 0; 1 0.5 0.3 0.1];
%      r = [1 1 1; 2 3 4];
%      n = quatrotate( q, r )
%
%   See also QUAT2DCM, QUATCONJ, QUATDIVIDE, QUATINV, QUATMOD,
%   QUATMULTIPLY, QUATNORM, QUATNORMALIZE.

%   Copyright 2000-2010 The MathWorks, Inc.

if any(~isreal(q(:)))
    error(message('aero:quatrotate:isNotReal1'));
end

if (size(q,2) ~= 4)
    error(message('aero:quatrotate:wrongDimension1'));
end

if any(~isreal(r(:)))
    error(message('aero:quatrotate:isNotReal2'));
end

if (size(r,2) ~= 3)
    error(message('aero:quatrotate:wrongDimension2'));
end

if (size(r,1) ~= size(q,1) && ~( size(r,1) == 1 || size(q,1) == 1))
    error(message('aero:quatrotate:wrongDimension3'));
end

dcm = quat2dcm(q);

if ( size(q,1) == 1 )
    % Q is 1-by-4
    qout = (dcm*r')';
elseif (size(r,1) == 1)
    % R is 1-by-3
    for i = size(q,1):-1:1
        qout(i,:) = (dcm(:,:,i)*r')';
    end
else
    % Q is M-by-4 and R is M-by-3
    for i = size(q,1):-1:1
        qout(i,:) = (dcm(:,:,i)*r(i,:)')';
    end
end