%INV Invert a quaternion
%
%	QI = inv(Q)
%
% Return the inverse of the unit-quaternion Q.
%

% Copyright (C) 2016  by Peter I. Corke

function qi = inv(q)

	qi = quaternion([q.s -q.v])/norm(double(q))^2;
