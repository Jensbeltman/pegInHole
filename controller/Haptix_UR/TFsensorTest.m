%% Set up the simulation and the matlab robotic toolbox model and place it in start position
% NB: Do this test withut gravity
clear
close all
clc

path('apimex',path)
path('Mujoco_lib',path)
dt=1/10;

robot_ = importrobot('./UR5/ur5_robot.urdf');
robot_.DataFormat = 'column';
robot_.show;

    % Rotation to convert between the robotics toolbox and the simulator
R_w_b = rotz(pi);
R_e_ee= quat2rotm([0.5 -0.5 0.5 -0.5]);
    

% connect to Haptix 
mj_close
mj_connect

% Set the start position 

q_start = [-pi/4 -pi/2 -pi/2 -pi/2 pi/2 0];
%q_start = [0 0 0 0 0 0];

JmoveM(q_start,2);

%% FT sensor test
    % Specify the desired position and orientation
pos_ = [0, -0.42, 0.0];    % Specify Position
ori_ = eye(3);              % Specify orientation
q_ = InverseKinematic(pos_, ori_, q_start');

tform = getTransform(robot_,q_,'ee_link','base_link')
x_ = kinjac_UR5Tcp(q_)

JmoveM(q_,2);
robot = RecMuJoCoData;
q_e = robot.JntPos;
x_ = kinjac_UR5Tcp(q_e)

%%
robot = RecMuJoCoData;
    q_e = robot.JntPos;
    qd_e = robot.JntVel;
    [x_e,R_]   = kinjac_UR5Tcp(q_e);
    robot.FTcp
    F_e = R_ * robot.FTcp % -> TRansform (only rotate)  sensor frame to baseframe [N]
    

%%
clc
pl = [];
Ftq = [];
for i = 1:20    
    display('----')
    i
    robot = RecMuJoCoData;
    q_e = robot.JntPos;
    qd_e = robot.JntVel;
    [x_e,R_]   = kinjac_UR5Tcp(q_e);
    robot.FTcp
    F_e = R_ * robot.FTcp % -> TRansform (only rotate)  sensor frame to baseframe [N]
    F_ = mj_get_force;
    

    pos = robot.CartPos';
    pl(:, end+1) = F_e';
    xd_out = [0; 0; -.1];
    Data = CmoveForUR5M(xd_out'.*1/10 ,1/10);
    
    for j = 1:size(Data.Ftcp,2)
         Ftq(:, end+1) = R_*Data.Ftcp(:,j);
        
    end
end
   
close all
subplot(2,1,1)
plot(pl(1,1:end)')
hold on 
plot(pl(2,1:end)')
hold on 
plot(pl(3,1:end)')
legend('x','y','z')
hold off

subplot(2,1,2)
plot(Ftq')
legend('x', 'y', 'z')


