clear
close all
clc

startup_rvc %Outcomment if you insstalled rvc toolbox differently
 % Specify path to libraries
path('apimex',path)
path('Mujoco_lib',path)

% Connect to Haptix and reset robot position
mj_close
mj_connect
mj_reset();
%% 
    % Get the current state of the robot
robot = RecMuJoCoData;
q0 = robot.JntPos;
[x0, R0] = kinjac_UR5Tcp1(q0);
T0 = MakeT(R0,x0);

    % Specify desired position and orientation
%x_d =  [-0.3 -0.3 0.5]';
%x_d = [ 0.3 -0.3 0.5]';
%x_d = [ 0.3  0.3 0.5]';
%x_d = [-0.3  0.3 0.5]';
%x_d = [ 0.1  0.1 0.5]';
%x_d = [-0.1 -0.1 0.6]
%x_d = [-0.2 -0.2 0.6]';
%x_d = [-0.2 -0.2 0.9]';
x_d = [-0.2 -0.2 0.1]';

R_d = roty(pi)*eye(3);

    % Desired pose
T_d = MakeT(R_d,x_d);

    % Number of steps in trajectory
n = 10;
    % Generate Cartesian trajectory from current to desired pose
tt = ctraj(T0, T_d, n);


%% PD control for position only
    % To plot
desired_ = [];
tau_ = [];
measured_ = [];

for i = 1:n % For each pose in trajectory
    T = tt(:,:,i);
    
    x_e = [tform2trvec(T)'];
    [desired, tau, measured] = PDGravityPos(x_e);
    
    desired_ = [ desired_ desired];
    tau_ = [tau_ tau];
    measured_ = [measured_ measured];
end
%% PD Control for both Position and Orientation
    % To plot
desired_ = [];
tau_ = [];
measured_ = [];

for i = 1:n % For each pose in trajectory
    T = tt(:,:,i);
    %eaa_ = tform2axang(T);
    %eaa_ = [eaa_(1:3) * eaa_(4)];
    
    %x_ = [tform2trvec(T)'; eaa_'];
    x_ = [T(1:3,4)];
    R_ = [T(1:3,1:3)];
    [desired, tau, measured] = PDGravity(x_,R_);
    
    desired_ = [ desired_ desired];
    tau_ = [tau_ tau];
    measured_ = [measured_ measured];
end
%% Plot Results
close all
plot(measured_')
hold on 
plot(desired_')

%legend('x', 'y' , 'z', 'theta_x', 'theta_y', 'theta_z', 'xd', 'yd', 'zd', 'theta_xd', 'theta_yd', 'theta_zd' )
legend('x', 'y' , 'z', 'xd', 'yd', 'zd' )
xlabel('Sample Number')
ylabel('Positional Value')
hold off

figure()
plot(tau_')
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6')
xlabel('Sample Number')
ylabel('Torque Value')
%% Old Outcommented Stuff
%applied.qfrc_applied
%applied.qfrc_applied = [0 0 0 0 0 0];
%mj_set_applied(applied);

%%%
%q_start1 = [0 -pi/2 -pi/2 -pi/2 0 0];
%q_start2 = [pi/2 -pi/2 -pi/2 -pi/2 0 0];
%%%
%for i = 1:2
%JmoveM(q_start1,2);
%end
%%
    %Con = mj_get_control;
    %tau = [0 0 0 0 0 0];
    %Con.ctrl = tau;
    %mj_set_control(Con)
%%
%for i=1:20
%    i
%    Con = mj_get_control
%    qdot = [0 0 0 0 0 0];
%    Con.ctrl = qdot
%    mj_set_control(Con)
    
%    Con1 = mj_get_control;
%    Con1.ctrl;
%    pause(0.1);
%end


%GetTcpFTsim
%q_start = [0 -pi/2 -pi/2 -pi/2 pi/2 0];
%JmoveM(q_start,2); 
%GetTcpFTsim
%%
%applied = mj_get_applied();
%applied.qfrc_applied = [0;0;0;0;0;0];
%mj_set_applied(applied);
%    robot = RecMuJoCoData;
%    q = robot.JntPos
%    qdot = robot.JntVel;
%%  
%applied = mj_get_applied();
%applied.qfrc_applied = [10;0;0;0;10;0];
%mj_set_applied(applied);
%mj_get_applied().qfrc_applied
% 
%     robot = RecMuJoCoData;
%     q = robot.JntPos
%     qdot = robot.JntVel;

%% Jacobian Transpose Control robot in Joint space

% q_start = q;
% q_desired = [0 -deg2rad(20) 0 -pi/2 -pi/2 0];
% q_desired = [0 -pi/2 -pi/2 -pi/2 pi/2 0];

%% J_Pi^li example slide 62 lecture 5 

% Inverse kinematics to find poses. 
% [x_start, R_start] = kinjac_UR5Tcp1(q_start);
% [x_desired, R_desired] = kinjac_UR5Tcp1(q_desired);
%x_desired = kinjac_UR5Tcp1(q_desired);


%%

 % Convert to euler 'ZYZ' angles
% Q = quaternion(R_desired,'rotmat', 'point');
% R_d = euler(Q,'ZYZ','point')
% 
% x_d = [x_desired; R_d'];
% PDGravity(x_d, dt);


%%

    % Convert to euler 'ZYZ' angles
% Q = quaternion(R_desired,'rotmat', 'point');
% R_d = euler(Q,'ZYZ','point')
% 
% x_d = [x_desired; R_d'];
% PDGravity(x_d, dt);
% 
% x_desired = [-0.3 -0.3 0.1]';
%     % Convert to euler 'ZYZ' angles
% R_desired = roty(pi)*eye(3);
% Q = quaternion(R_desired,'rotmat', 'point');
% R_d = euler(Q,'ZYZ','point')
% 
% x_d = [x_desired; R_d'];
% PDGravity(x_d, dt);

%%
% Controller
%x_d = [x_desired; rotm2eul(R_desired,'ZYZ')'];
% x = x_start;

% T_desired = [R_desired x_desired; [0 0 0] 1];
% 
% J = PDGravity(T_desired, dt);

%J = PDGravity(x_d, dt);
%J = PDGravity(x_desired, dt);


%x_desired = [0.3 -0.3 0.1]';
% the function generates a lin. path from mesured joint pos to desired (q)
% in time spec. in [s]

% Ja = JacobianA(q_desired);

% JmoveM(q_start,2);
% JmoveM(q_desired,2);



%% Use forward kinematic to calculate the desired position
% It doesnt work as expected
    % Convert to euler 'ZYZ' angles
% [p_start R_start J_start] = kinjac_UR5Tcp(q_start);
% Q_start = quaternion(R_start,'rotmat','point');
% eulerAngles_start = euler(Q_start,'ZYZ','point')';
% x_start = [p_start ; eulerAngles_start];

    % Convert to euler 'ZYZ' angles
% [p_desired R_desired J_desired] = kinjac_UR5Tcp(q_desired);
% Q_desired = quaternion(R_desired,'rotmat','point')
% eulerAngles_desired = euler(Q_desired,'ZYZ','point')';
% x_desired = [p_desired ; eulerAngles_desired];

%% Move robot in Joint space

% q_desired=[0 pi/2 0 0 0 0];
% the function generates a lin. path from mesured joint pos to desired (q)
% in time spec. in [s]
% JmoveM(q_desired,2);

%% Move robot in Joint space

% q_desired=[0 -pi/2 0 0 0 0];
% the function generates a lin. path from mesured joint pos to desired (q)
% in time spec. in [s]
% JmoveM(q_desired,2);

%% move in Cart. space
% move for a distance (vector), in spe. time [s], orientation is constant
% CmoveForUR5M([0 0 -0.2],3);
% move to a T frame in Crt space.
% CmoveUR5M(T1,4);

mj_close