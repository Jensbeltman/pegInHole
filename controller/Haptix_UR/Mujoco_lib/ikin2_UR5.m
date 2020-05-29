function [qi, xi] = ikin2_UR5(x,R,q)
Qr = quaternion(R);
qi = q(:);
K = 0.9;
count = 0;
[xi,Ri,Ji] = kinjac_UR5Tcp(qi);
%position controller
xc = K*(x(:)-xi(:));
%orientation controller
Qi = quaternion(Ri);
Qc = Qr*inv(Qi);
realQcv=real(Qc.v);
qdi = pinv(Ji)*K*[xc(:);realQcv(:)];
%integration
qi = qi + qdi;
%check
count = count + 1;
% end;