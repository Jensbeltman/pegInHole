%MTIMES Multiply for quaternions
%
% Invoked by the * operator
%
% q1*q2	standard quaternion multiplication
% q1*v	rotate vector v by quaternion
% v*q1	rotate vector v by quaternion
% a*q1	multiply scalar and quaternion
% q1*a	multiply scalar and quaternion

% $Log: mtimes.m,v $
% Revision 1.3  2002/04/01 12:06:47  pic
% General tidyup, help comments, copyright, see also, RCS keys.
%
% $Revision: 1.3 $
%
% Copyright (C) 1999-2002, by Peter I. Corke
%
% Modified 2015 by Leon (removed mirroring if s<0)
% Modified 2016 by Leon (added multiplication with scalar)
%
function Qp = mtimes(Q1, Q2)

if isa(Q1, 'quaternion')
    s1=Q1.s;
    v1=Q1.v;
    if isa(Q2, 'quaternion')
        s2=Q2.s;
        v2=Q2.v;
        Qp=quaternion([s1*s2-v1*v2' s1*v2+s2*v1+cross(v1,v2)]);
    elseif all(size(Q2)==[3 1])
        qpx=Q1*quaternion([0 Q2'])*inv(Q1);
        Qp=qpx.v';
    elseif all(size(Q2)==[1 3])
        qpx=Q1*quaternion([0 Q2])*inv(Q1);
        Qp=qpx.v;
    elseif isscalar(Q2)
        Qp=quaternion([Q2*s1 Q2*v1]);
    else
        error('Wrong inputs')
    end
elseif isa(Q2, 'quaternion')
    s2=Q2.s;
    v2=Q2.v;
    if all(size(Q1)==[3 1])
        qpx=inv(Q2)*quaternion([0 Q1'])*Q2;
        Qp=qpx.v';
    elseif all(size(Q1)==[1 3])
        qpx=inv(Q2)*quaternion([0 Q1])*Q2;
        Qp=qpx.v;
    elseif isscalar(Q1)
        Qp=quaternion([Q1*s2 Q1*v2]);
    else
        error('Wrong inputs')
    end
else
    error('Wrong inputs')
end
