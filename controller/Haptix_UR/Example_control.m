%% example script
clear
close all
clc

% (NB: remove quaternion if it doesnt work)

path('apimex',path)
path('Mujoco_lib',path)
dt=1/100;

% connect to Haptix 
mj_close
mj_connect
%%
clear 
clc

path('RoboticToolboxUR5',path)
robot = importrobot('ur5_robot.urdf');
robot.DataFormat = 'col';
robot.show
q_start = [0 -pi/2 -pi/2 -pi/2 pi/2 0]';

links =string( {'shoulder_link'; 'upper_arm_link'; 'forearm_link'; ... 
    'wrist_1_link'; 'wrist_2_link'; 'wrist_3_link'; 'ee_link' });

for i = 1:6
    links(i)
    geoJacob = geometricJacobian(robot,q_start,links(i))
end



%%
q_start1 = [0 -pi/2 -pi/2 -pi/2 0 0];
q_start2 = [pi/2 -pi/2 -pi/2 -pi/2 0 0];


JmoveM(q_start1,2);
%%
for i = 1:2
JmoveM(q_start1,2);
JmoveM(q_start2,2);
end
%%
for j=1:10
CmoveForUR5M([-0.2 -0.2 0],0.1);
CmoveForUR5M([0.2 0.2 0],1);
end
%%
    Con = mj_get_control;
    qdot = [0 0 0 0 0 0];
    Con.ctrl = [qdot'; q_start' ]
    mj_set_control(Con)
%%
for i=1:20
    i
    Con = mj_get_control;
    qdot = [100 0 0 0 0 0];
    Con.ctrl = [qdot'; q_start' ]
    mj_set_control(Con)
    
    Con1 = mj_get_control;
    Con1.ctrl;
    pause(0.1);
end


%GetTcpFTsim
%q_start = [0 -pi/2 -pi/2 -pi/2 pi/2 0];
%JmoveM(q_start,2);
%GetTcpFTsim
%%
applied = mj_get_applied();
applied.qfrc_applied = [0;0;0;0;0;0];
mj_set_applied(applied);
    robot = RecMuJoCoData;
    q = robot.JntPos
    qdot = robot.JntVel;
%%  
applied = mj_get_applied();
applied.qfrc_applied = [0;0;0;0;1000;0];
mj_set_applied(applied);
mj_get_applied().qfrc_applied

    robot = RecMuJoCoData;
    q = robot.JntPos
    qdot = robot.JntVel;

%% Jacobian Transpose Control robot in Joint space
clc

q_start = [0 -pi/2 -pi/2 -pi/2 0 0];
q_desired =[0 -pi/2 -pi/2 -pi/2 pi/2 0];


JmoveM(q_desired,2);
    robot = RecMuJoCoData;
    x_desired = [robot.CartPos; quat2eul(robot.CartOri','ZYZ')' ];
    
    
    
JmoveM(q_start,2);
    robot = RecMuJoCoData;
    x_start = [robot.CartPos; quat2eul(robot.CartOri','ZYZ')' ];
    
    
%%
clc
JmoveM(q_start,2);

    % Controller
x_d = x_desired;
x = x_start;

g = @(q) [0 0 0 0 0 0]'; 
Kp = 1;
Kd = 0.1;

%for t = 1:dt:2
        % Measurements from robot manipulator
    robot = RecMuJoCoData;
    q = robot.JntPos;
    qdot = robot.JntVel;
    x_e = [robot.CartPos; quat2eul(robot.CartOri','ZYZ')' ];
        % Calculate the error
    x_tilde = x_d - x_e

        % Calculate the Jacobian
    [U S V] = svd(JacobianA(q));
    Ja_T = V'*S'*U';
    
        % Calculate the input torque
    u = g(q) + Ja_T*Kp*x_tilde-Kd*qdot
    
%end



% the function generates a lin. path from mesured joint pos to desired (q)
% in time spec. in [s]

%Ja = JacobianA(q_desired);

%JmoveM(q_start,2);
%JmoveM(q_desired,2);



%% Use forward kinematic to calculate the desired position
% It doesnt work as expected
    % Convert to euler 'ZYZ' angles
[p_start R_start J_start] = kinjac_UR5Tcp(q_start);
Q_start = quaternion(R_start,'rotmat','point');
eulerAngles_start = euler(Q_start,'ZYZ','point')';
x_start = [p_start ; eulerAngles_start];

    % Convert to euler 'ZYZ' angles
[p_desired R_desired J_desired] = kinjac_UR5Tcp(q_desired);
Q_desired = quaternion(R_desired,'rotmat','point')
eulerAngles_desired = euler(Q_desired,'ZYZ','point')';
x_desired = [p_desired ; eulerAngles_desired];

%% Move robot in Joint space

q_desired=[0 pi/2 0 0 0 0];
% the function generates a lin. path from mesured joint pos to desired (q)
% in time spec. in [s]
JmoveM(q_desired,2);

%% Move robot in Joint space

q_desired=[0 -pi/2 0 0 0 0];
% the function generates a lin. path from mesured joint pos to desired (q)
% in time spec. in [s]
JmoveM(q_desired,2);

%% move in Cart. space
% move for a distance (vector), in spe. time [s], orientation is constant
CmoveForUR5M([0 0 -0.2],3);
% move to a T frame in Crt space.
%CmoveUR5M(T1,4);

mj_close