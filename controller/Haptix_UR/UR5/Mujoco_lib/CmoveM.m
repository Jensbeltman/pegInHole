%==========================================================================
function Data = CmoveM(Td,time,tcp)
%==========================================================================
% Calculates cartesian trajectory Ti
% Calculates joint trajectory qi
% Executes joint motion qi
% parameters
%   Td - desired Carthesian pose in world coordinates
%   t - duration
%   tcp - tool centre point row vector 3x1


 RobotBase = [ 1     0     0     0
              0     1     0     0
              0     0     1     0
              0     0     0     1];

dt=1/100;
%prepare cartesian trajectory
if nargin < 3
    tcp = [0 0 0];
end
%calculate robot T with specified tcp in robot base
Con = mj_get_control;
robot = RecMuJoCoData;
q = robot.JntPos;
[p,R] = kinjac_lwr(q,tcp);
T0 = MakeT(R,p); %in robot base

disp('cmove prepare..')

robot = RecMuJoCoData;
q = robot.JntPos; %starts from last measured position - for initial IKIN
N = round(time/dt);
Ti = ctraj(T0,Td,N);
%prepare joint trj
qi = zeros(size(Ti,3),7);
for i = 1:size(Ti,3)
    x = Ti(1:3,4,i)';
    R = Ti(1:3,1:3,i);
    q = ikin_lwr(x,R,tcp',q);
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
    Con.ctrl(1:7) = qi(i,:);
    Con.ctrl(8:14)= qdi(i,:);
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
