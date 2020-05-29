%==========================================================================
function Data = CmoveForUR5M(v,time)
%==========================================================================
% Calculates cartesian trajectory Ti
% Calculates joint trajectory qi
% Executes joint motion qi
% parameters
%   v - velocity [p, \phi] ('zyx') in world coordinates
%   t - duration
%   tcp - tool centre point row vector 3x1

RobotBase = [ 1     0     0     0
              0     1     0     0
              0     0     1     0.445
              0     0     0     1];

dt=1/1000;
%dt=1/100;
%prepare cartesian trajectory
if nargin < 3
    tcp = [0 0 0];
end
%calculate robot T with specified tcp in robot base
Con = mj_get_control;
robot = RecMuJoCoData;
q = robot.JntPos;
[p,R] = kinjac_UR5Tcp(q);
T0 = MakeT(R,p); %in robot base
%rotate displacement to robot base
db = RobotBase(1:3,1:3)'*(v(1:3).*time)';
do = axang2rotm([v(4:6)./norm(v(4:6)) norm(v(4:6))*time]);
T1 = T0;
T1(1:3,4) = T0(1:3,4) + db;
T1(1:3,1:3) = T0(1:3,1:3)*do;
% 
% display('T')
% T0
% T1


%disp('ccmove prepare..')

robot = RecMuJoCoData;
q = robot.JntPos; %starts from last measured position - for initial IKIN
N = round(time/dt);
Ti = ctraj(T0,T1,N);

%prepare joint trj
qi = zeros(size(Ti,3),6);
for i = 1:size(Ti,3)
    x = Ti(1:3,4,i)';
    R = Ti(1:3,1:3,i);
    q = ikin2_UR5(x,R,q);
    %CheckJointLimits(q);
    qi(i,:) = q;
end
%% calculate velocities
qdi = diff(qi)/dt;
qdi = [qdi;qdi(end,:)];

%execute
%disp('cmove exec..')
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
%disp('done.');
