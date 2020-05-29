%LOG Logarithm of a quaternion
%
%	Ql = log(Q)
%
% Returns the logarithm of the quaternion Q.
%
% Copyright (c) 2016 by IJS Leon Zlajpah
% Version: 1.0
%

function Ql=log(Q)

vn=norm(Q.v);
if abs(vn)<=1e-12*abs(Q.s)
    if(Q.s<0.0) 
      error('Infinitely many solutions for log');
    end
    Ql=quaternion([log(Q.s) 0 0 0]);
else
    w=atan2(vn,Q.s);
    Ql=quaternion([log(Q.s/cos(w)) w*Q.v/vn]);
end
