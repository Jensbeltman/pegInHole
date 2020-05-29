%EXP Exponent of a quaternion
%
%	Qe = exp(Q)
%
% Returns the exponent of the quaternion Q.
%
% Copyright (c) 2016 by IJS Leon Zlajpah
% Version: 1.0
%

function Qe=exp(Q)

vn=norm(Q.v);
if abs(vn)<=1e-12*abs(Q.s)
    Qe=quaternion([exp(Q.s) 0 0 0]);
else
    e=exp(Q.s);
    Qe=quaternion([e*cos(vn) e*sin(vn)*Q.v/vn]);
end