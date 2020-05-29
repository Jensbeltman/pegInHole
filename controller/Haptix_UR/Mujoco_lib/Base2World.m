function TW = Base2World(TB,T0)
% functon converts robot base coordinates to the world coordinates
% T0 denotes robot mounting displacement
pB = TB(1:3,4);
RB = TB(1:3,1:3);
p0 = T0(1:3,4);
R0 = T0(1:3,1:3);

pW = R0*pB + p0;
RW = R0*RB;

TW = MakeT(RW,pW');