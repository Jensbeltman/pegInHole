function TB = World2Base(TW,T0)
% functon converts world coordinates to robot base coordinates
% T0 denotes robot mounting displacement
pW = TW(1:3,4);
RW = TW(1:3,1:3);
p0 = T0(1:3,4);
R0 = T0(1:3,1:3);

%world to robot base
pB = R0'*(pW-p0);
RB = R0'*RW;

TB = MakeT(RB,pB');