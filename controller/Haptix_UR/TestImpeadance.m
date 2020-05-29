%% Set up the simulation and the matlab robotic toolbox model and place it in start position
clear
close all
clc


path('apimex',path)
path('Mujoco_lib',path)
dt=1/10;

robot_ = importrobot('./UR5/ur5_robot.urdf');
robot_.DataFormat = 'column';
%robot_.show;

% connect to Haptix 
mj_close; mj_connect

% Set the start position 
Initial_guess_ = [-pi/4 -pi/2-0.5 -pi/2 -pi/2 pi/2 0];
    % Specify the desired position and orientation
pos_ = [0 ,-0.5, 0.3];    % Specify Position
ori_ = eye(3)*roty(pi/2);              % Specify orientation
q_ = InverseKinematic(pos_, ori_, Initial_guess_');

JmoveM(q_,2);

%%
% Set the start position 
Initial_guess_ = [-pi/4 -pi/2-0.5 -pi/2 -pi/2 pi/2 0];
    % Specify the desired position and orientation
pos_ = [0.3 ,-0.3, 0.0];    % Specify Position
ori_ = eye(3);              % Specify orientation w.r.t a coordinate system that has the z-pointing down into the table
q_ = InverseKinematic(pos_, ori_, Initial_guess_');

JmoveM(q_,2);

%% Impedance script
desired_pos = [0.3; -0.4; -0.1];        % Desired position [x y z] [m]
desired_ori = rotm2axang(roty(pi))';% Desired orientation ['zyx'] [rad] - x along W: -x, y along y and z along -z
desired_ori = desired_ori(1:3)*desired_ori(4);

desired_pos_acc = [0; 0; 0];     % Desired positional acceleration [x y z] [m/s^2]
desired_ori_acc = [0; 0; 0];     % Desired positional acceleration [EAA] [rad/s^2]

desired_force  = [0; 0; 1];      % Desired force  [x y z] [N]
desired_torque = [0; 0; 0];      % Desired torque ['zyx'] [Nm]
    
x_d = [desired_pos ; desired_ori];
xdd_d = [desired_pos_acc ; desired_ori_acc];
F_d = [desired_force ; desired_torque];

out = ImpedanceControl3(x_d, xdd_d, F_d);

%% ################################################################
%%

Ftcp_C = [];
Ftcp_I = [];
pos = []
for i = 1:size(out,2)
    Ftcp_C = [Ftcp_C out{i}.Ftcp_C];
    Ftcp_I = [Ftcp_I out{i}.Ftcp_I];
    pos = [pos out{i}.pos];
end

close all
subplot(2,1,1)
plot(Ftcp_C(:,:)')
legend('x', 'y', 'z')
ylabel('Force [N]')
xlabel('Time step')
title('Force: Velocity controller')

subplot(2,1,2)
plot(Ftcp_I(:,:)')
legend('x', 'y', 'z')
ylabel('Force [N]')
xlabel('Time step')
title('Force: Impedance controller')

mean(Ftcp_I(:,1:10)')


figure()
subplot(1,3,1)

plot(pos(1:3,:)')
yline(-0.4)
yline(-0.1)
legend('x', 'y', 'z', 'y_{ref}', 'z_{ref}')
ylabel('Position')
xlabel('Time step')
title('Position: Impedance controller')
ylim([-0.5 0.4])


subplot(1,3,2)
plot(pos(4:6,:)')
yline(-0.4)
legend('rx', 'ry', 'rz')
ylabel('Position')
xlabel('Time step')
title('Orientation: Impedance controller')
ylim([-0.5 0.4])

subplot(1,3,3)
plot(Ftcp_I(:,:)')
legend('x', 'y', 'z')
ylabel('Force [N]')
xlabel('Time step')
title('Force: Impedance controller')



%%
figure()
ylabel('Force in z-axis [N]')
xlabel('Time step')
title('Velocity Controller')
hold on
for i=1:10
    plot(out{i}.Ftcp_C(3,:))
end
hold off


%%
clc
pl = []

for i = 1:10
    display('----')
    i
    robot = RecMuJoCoData;
    F_e = robot.FTcp;
    pos = robot.CartPos'
    pl(:, end+1) = F_e'*100;
    xd_out = [0; 0; 1];
    out = CmoveForUR5M(xd_out'.*0.1 ,0.1);
end
   
close all
plot(pl(1,1:end)')
hold on 
plot(pl(2,1:end)')
hold on 
plot(pl(3,1:end)')
legend('x','y','z')


%% Other test
q_s     = [0.8 -2.25 -1.95 -0.6 pi/2 0];
%q_d     = [0.9+pi -2.15 -1.95 -0.6 pi/2 0];
%q_d     = [0.9+pi -2.20 -2.25 -0.35 pi/2 0];
q_d     = [0.9+pi -2.22 -2.02 -0.45 pi/2 0];
%q_start = [pi/4 -pi/2 pi/2 -pi/2 -pi/2 0];


mdl_ur5

ur5.ikine(trans([ 0.4; 0.4; 0.2]))


%q_e = ikin2_UR5([ 0.4; 0.4; 0.2], eye(3), q_d)

%JmoveM(q_s,2);
JmoveM(q_e,2);




%%
clc

rotm1 = rotz(pi/2)*roty(pi/2)

eaa = rotm2axang(rotm1)
eaa = eaa(1:3) .* eaa(4)

l_eaa = norm(eaa)
axang2rotm([eaa./l_eaa l_eaa])