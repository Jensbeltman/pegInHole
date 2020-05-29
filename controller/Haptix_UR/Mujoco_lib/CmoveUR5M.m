%==========================================================================
function CmoveUR5(xd,td)
%==========================================================================
% cartesian interpolation move from the current poistion to the
% frame (T) in (t) seconds
t=0.0;
dt=1/100;
disp('Cmove..')
N = round(td/dt);

RobotBase = [ 1     0     0     0
              0     1     0     0
              0     0     1     0.445
              0     0     0     1];


% get robot position
Con = mj_get_control;
robot = RecMuJoCoData;
q = robot.JntPos;
[p,R] = kinjac_UR5Tcp(q);
T0 = MakeT(R,p); %in robot base
%rotate displacement to robot base
db = RobotBase(1:3,1:3)'*d';
T1 = T0;
T1(1:3,4) = T0(1:3,4) + db;

disp('ccmove prepare..')

robot = RecMuJoCoData;
q = robot.JntPos; %starts from last measured position - for initial IKIN
N = round(time/dt);
Ti = ctraj(T0,T1,N);
%prepare joint trj
qi = zeros(size(Ti,3),6);
for i = 1:size(Ti,3)
    x = Ti(1:3,4,i)';
    R = Ti(1:3,1:3,i);
    q=ikin2_UR5(x,R,q)
    %CheckJointLimits(q);
    qi(i,:) = q;
end
%% calculate velocities
qdi = diff(qi)/dt;
qdi = [qdi;qdi(end,:)];

%execute
disp('cmove exec..')
tn=0;
st = tic;
for i = 1:size(qi,1)
    Con.ctrl(1:6) = qi(i,:);
    Con.ctrl(7:12)= qdi(i,:);
    mj_set_control(Con);
    robot = RecMuJoCoData;
    SeDat(:,i) = robot.CartPos;
    SeOri(:,i) = robot.CartOri;
    FT(:,i) = robot.FTcp;
    tn = tn+dt;
    if tn>toc(st)
        pause(tn-toc(st))
    end
end
Data.CartPos=SeDat;
Data.CartOri = SeOri;
Data.Ti = Ti;
Data.Ftcp = FT;
disp('done.');
