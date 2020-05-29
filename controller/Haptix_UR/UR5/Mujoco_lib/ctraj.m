%CTRAJ Compute a Cartesian trajectory between two points
%
% 	TC = CTRAJ(T0, T1, N)
%	TC = CTRAJ(T0, T1, R)
%
% Returns a Cartesian trajectory TC from point T0 to T1.  The number
% of points is N or the length of the given path distance vector R.
%
% In the first case the points are equally spaced between T0 and T1.
% In the second case R gives the distance along the path, and the 
% elements of R must be in the range [0 1].
%
% Each trajectory is a 4x4xn matrix, with the last subscript being the
% point index.
%
% SEE ALSO: TRINTERP, QINTERP, TRANSL.
%

% Copyright (C) 1993-2002, by Peter I. Corke

% MOD. HISTORY
% 	12/94	track changes to trinterp()
% 	4/99	add object support
% 	6/99	init tt to zeros rather than [], problem with cat() v 5.3
% $Log: ctraj.m,v $
% Revision 1.3  2002/04/14 10:11:38  pic
% Changed see also info.
%
% Revision 1.2  2002/04/01 11:47:11  pic
% General cleanup of code: help comments, see also, copyright, remnant dh/dyn
% references, clarification of functions.
%
% $Revision: 1.3 $


function tt = ctraj(t0, t1, n)
	if length(n) == 1,
		i = 1:n;
		r = (i-1)/(n-1);
	else
		r = n(:)';
		n = length(r);
	end

	if any(r> 1) | any(r<0),
		error('path position values (R) must 0<=R<=1)')
	end
	tt = zeros(4,4,0);

	for R=r,
		tt = cat(3, tt, trinterp(t0, t1, R));
	end

	
