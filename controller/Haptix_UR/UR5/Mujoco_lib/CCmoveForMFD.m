%==========================================================================
function Data = CCmoveForMFD(d,time,tcp)
%==========================================================================
% Cartesian interpolation in modes 10,30
% Calculates cartesian trajectory Ti
% Calculates joint trajectory qi
% Executes joint motion qi
% parameters
%   d - displacemet in world coordinates
%   t - duration
%   tcp - tool centre point row vector 3x1
Des = 0.1;

% RobotBase=[   -0.8398    0.4610    0.2868         0
%               -0.4330   -0.2500   -0.8660   -0.0765
%               -0.3276   -0.8514    0.4096    0.9400
%                     0         0         0    1.0000];
CompFTs = ComFT;
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
qi = zeros(size(Ti,3),7);
for i = 1:size(Ti,3)
    x = Ti(1:3,4,i)';
    R = Ti(1:3,1:3,i);
    q = ikin_lwr(x,R,tcp',q);
    %CheckJointLimits(q);
    qi(i,:) = q;
end;
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
    ft =GetTcpFTsim;
    FTdat(:,i) = (ft-CompFTs)';
    SeDat(:,i) = robot.CartPos;
    SeOri(:,i) = robot.CartOri;
    
    if abs(ft(1:3)) > Des
        disp('Active force reached')
       break; 
    end
    
    tn = tn+dt;
    if tn>toc(st)
        pause(tn-toc(st))
    end
end
Data.FT = FTdat;
Data.CartPos=SeDat;
Data.CartOri = SeOri;
Data.Ti = Ti;
disp('done.');
