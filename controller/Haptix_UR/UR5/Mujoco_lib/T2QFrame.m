function QFrame = T2QFrame(T);
%converts homogenous trasformation frame to quaternion frame
x = T(1:3,4);
R = T(1:3,1:3);
Q = r2q(R);
% Q = quaternion(R);
QFrame = [x' Q.d];