function T = QFrame2T(QF)

if size(QF) == [1 7]
    QF = QF';
end
if size(QF) == [7 1]
x = QF(1:3);
R = q2r(QF(4:7));
T = [R x; 0 0 0 1];
else
    disp('bad quaternion data')
end
