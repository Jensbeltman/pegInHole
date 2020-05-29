function ForceControlSim(Fd,t,tcp, RobotBase)
dt=1/100;
if nargin < 3
    tcp = [0 0 0];
end
[b,a] = butter(1,0.01);
disp('Force control...');
N=t/dt;
Kf = 0.000005;
Kd = 0.000025;
FFerr = [];
Fcap = [];
Con = mj_get_control;
robot = RecMuJoCoData;
q = robot.JntPos;
tn=0;
st = tic;
Ferr0=[0 0 0]';
for i = 1:N
    %get the robot frame
    Tr = RobotTSim('c');
    R = Tr(1:3,1:3);
    p = Tr(1:3,4);
    %get the force
    FT = GetTcpFTsim;
    Fm = [FT(1),FT(2),-FT(3)];
    Fm= filter(b,a,Fm)
    Ferr = (Fd-Fm)';
    dFerr = Ferr -Ferr0;
    Ferr0 = Ferr;
    %dead zone and limit
    for i = 1:3
        if abs(Ferr(i)) < 2, Ferr(i) = 0; end
        if Ferr(i) > 20, Ferr(i) = 20; end
        if Ferr(i) < -20, Ferr(i) = -20; end
    end
    %do the offset in tool coordinates
    control = R*(Kf*Ferr + Kd*dFerr);
    p = p - control;
    NewT = MakeT(R,p);
    %To the base ..
%      NewTb = World2Base(NewT,RobotBase);
     NewTb=NewT;
    %tool offset
    %Tr = Tool2Robot(NewTb);
    x = NewTb(1:3,4)';
    R = NewTb(1:3,1:3);
    qn = ikin_lwr(x,R,tcp',q);
%      qdi = diff(qn')/dt;
    %% send to sim
    Con.ctrl(1:7) = qn;
%     Con.ctrl(8:14)= qdi;
    mj_set_control(Con);
    pause(0.1)
    tn = tn+dt;
%     if tn>toc(st)
%         pause(tn-toc(st))
%     end
    
%     FFerr = [FFerr; Ferr];
%     Fcap = [Fcap; Fm];
end
disp('done.');
