%QINTERP Interpolate rotations expressed by quaternion objects
%
%	QI = qinterp(Q1, Q2, R)
%
% Return a unit-quaternion that interpolates between Q1 and Q2 as R moves
% from 0 to 1.  This is a spherical linear interpolation (slerp) that can
% be interpretted as interpolation along a great circle arc on a sphere.
%
% If r is a vector, QI, is a cell array of quaternions, each element
% corresponding to sequential elements of R.
%
% See also: CTRAJ, QUATERNION.

% MOD HISTORY
% 2015-08-19    fixed selection of shortest path - see http://www.3dgep.com/understanding-quaternions/#Quaternion_Dot_Product (Leon)
% 2015-08-19    fixed if r is vector; if r<0 long-way interpolation  (Leon)
% 2/99 convert to use of objects
% $Log: qinterp.m,v $
% Revision 1.3  2002/04/14 11:02:54  pic
% Changed see also line.
%
% Revision 1.2  2002/04/01 12:06:48  pic
% General tidyup, help comments, copyright, see also, RCS keys.
%
% $Revision: 1.3 $
%
% Copyright (C) 1999-2002, by Peter I. Corke

function q = qinterp(Q1, Q2, r)
%#codegen

q1 = double(Q1);
q2 = double(Q2);


if any(abs(r)>1),
    error('Range error: abs(r)=<1');
end

if (q1*q2')<0
    q2=-q2;
end

if length(r) == 1,
    if r<0
        q2=-q2;
        r=-r;
    end
    theta = acos(q1*q2');
    if theta == 0,
        q = Q1;
    elseif theta == 1,
        q = Q2;
    else
        q = quaternion( (sin((1-r)*theta) * q1 + sin(r*theta) * q2) / sin(theta) );
    end
else
    q = cell(size(r));
    count = 1;
    for R=r(:)',
        if r<0 
            q2=-q2;
            R=-R;
        end
        theta = acos(q1*q2');
        if theta == 0,
            qq = Q1;
        elseif theta == 1,
            qq = Q2;
        else
            qq = quaternion( (sin((1-R)*theta) * q1 + sin(R*theta) * q2) / sin(theta) );
        end
        q{count} = qq;
        count = count + 1;
    end
end
