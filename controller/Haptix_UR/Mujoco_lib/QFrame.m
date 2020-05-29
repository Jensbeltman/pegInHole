function QF = QFrame(T)

pos = T(1:3,4);
Q = double(Quaternion(T));
QF = [pos;Q(1);Q(2);Q(3);Q(4)];
