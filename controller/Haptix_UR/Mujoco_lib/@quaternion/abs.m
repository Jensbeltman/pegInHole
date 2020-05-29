%ABS Absolute value of a quaternion
%
%	a = abs(Q)
%
% Returns the inverse of the quaternion Q.
%
% Copyright (c) 2016 by IJS Leon Zlajpah
% Version: 1.0
%

function a = abs(q)
	a = norm(double(q));
