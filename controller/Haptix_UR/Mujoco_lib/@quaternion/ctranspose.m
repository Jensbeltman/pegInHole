function q=ctranspose(q1)
%TRANSPOSE   Overloaded for quaternions
%   q = q1'
%   q = TRANSPOSE(q1)
%
%   Example:
%      q1 = quaternion(rotz(pi/4))
%      q = q1'
%
%   See also TRANSPOSE.

%   Copyright 2008 Leon Zlajpah
%   $Revision: 1.0 $  $Date: 2008/02/14 6:45:00 $

q=quaternion([q1.s -q1.v]);