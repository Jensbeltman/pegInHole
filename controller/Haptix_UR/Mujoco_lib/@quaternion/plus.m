function q=plus(q1,q2)
%ADDITION   Overloaded for quaternions
%   q = q1+q2
%

%   Copyright 2008 Leon Zlajpah
%   $Revision: 1.0 $  $Date: 2008/02/14 6:45:00 $

if isa(q2,'quaternion')
    q=quaternion([q1.s+q2.s q1.v+q2.v]);
end