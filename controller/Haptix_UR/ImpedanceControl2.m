% For this case, you just need to make sure that you are maintaining contact. 
% That means you don't really need stiffness (K=0), and then you want to push with a force 
% (F_target != 0). Then you would need to find a suitable inertia (M) and damping (D). 

% Only think about inertia and gravity - no curelias 

% Steer the robot to be place at position [x y z R P Y] and apply the force
% 
function out = ImpedanceControl(desired_position, desired_acceleration, desired_force)
    % Orientation is in EAA
    % Get the robt 
Con = mj_get_control;

    % Crash if dt is too low
dt = 1/10;

    % Tolerance
tol_pos = [10^(-3); 10^(-3); 10^(-3); 10^(-2); 10^(-2); 10^(-2)];
tol_acc = [10^(-2); 10^(-2); 10^(-2)];
tol_F   = [10^(-2); 10^(-2); 10^(-2); 10^(-2); 10^(-2); 10^(-2)]; 
tol     = [tol_pos; tol_acc; tol_F];

    % Stiffness vector
K_pos = ones(3,1) .*1;% [500; 500; 500];
K_ori = ones(3,1) * 0;
K = [K_pos; K_ori]; 
    % Inertia vector
M_pos = ones(3,1) * 100;
M_ori = ones(3,1) * 1;
M = [M_pos; M_ori]; % Inertia [kg] 
    % Damping vector
D_pos = ones(3,1) * 500;
D_ori = ones(3,1) * 1;
D = [D_pos; D_ori]; 

% Desired/target (3x1)
x_d = desired_position;      
xdd_d = desired_acceleration;
F_d = desired_force;

xd_e = [0;0;0;0;0;0];

out = {};

        % Repeat
% while (any( [x_e; xdd_e; F_e] > tol))

% tn=0;
% st = tic;
%for i= 1:100
    
    
i = 1;
[x_e, xd_e, xdd_e, FT_e] = update_state(xd_e, dt);
%while (~all( abs(x_d - x_e) < tol_pos) || ~all( abs(F_d - FT_e) < tol_F ))
while (1)
display('_______________')
    
%     st_ = tic;
        % Save the old measurement to the acceleration can be derived
    [x_e, xd_e, xdd_e, FT_e] = update_state(xd_e, dt);
    [
    x_e'
    x_d'
    xd_e'
    xdd_e'
    xdd_d'
    ]
    
            % Forces
    forces = FT_e - desired_force;
    
        %Position
    pos_stiffness = K(1:3).* (x_d(1:3) - x_e(1:3));
    pos_inertia = M(1:3).* (xdd_d(1:3) - xdd_e(1:3));
    pos_damping = D(1:3).*(- xd_e(1:3));
    xd_pos_target = (pos_stiffness + pos_inertia + pos_damping + forces(1:3))./ D(1:3);

        % Orientation
    rot_stiffness = K(4:6).* cross(x_d(4:6), x_e(4:6));
    rot_inertia = M(4:6).* cross(xdd_d(4:6), xdd_e(4:6));
    rot_damping = D(4:6).* cross(zeros(3,1), xd_e(4:6));
    rot_damping = [0;0;0];
    xd_rot_target = (rot_stiffness + rot_inertia + rot_damping + forces(4:6))./ D(4:6);

    
        % Displace the end-effector in [x y z \phi]
    xd_target = [xd_pos_target; xd_rot_target];
    xd_target'
    
    Data = CmoveForUR5M_ori(xd_target' ,dt);


    %t.Ftcp_C = Data.Ftcp;
    %t.Ftcp_I = FT_e(1:3); 
    
%    out(i) = {t};

    
%     if(x_d - x_e < tol_pos)
%         display('Tol')
%         %return 
%     end
if i == 50
    return
end
i = i + 1;
end


% NULL punkt offset
% F_e = R_ * robot.FTcp % -> TRansform (only rotate)  sensor frame to baseframe [N]
% T_e = R_ * robot.TTcp % -> TRansform (only rotate)  sensor frame to baseframe [N]


end


function [x_e, xd_e, xdd_e, FT_e] = update_state(xd_e_old, dt)
% Measured - Joint position, Joint velocity and TF at end effector
robot = RecMuJoCoData;
q_e = robot.JntPos;
qd_e = robot.JntVel;

% Measure the position, velocity and acceration at the end-effector. (3x1)
[x_e,R_]   = kinjac_UR5Tcp(q_e);    % Measured in base coordinates
R = rotm2axang(R_); % This might be wrong
R = R(1:3)*R(4);
x_e = [x_e; R'];

    % Get the velocity an convert it to EAA 
xd_e_  = JacobianA(q_e) * qd_e;  % Euler 'zyx'
EAAd_e = rotm2axang(eul2rotm(xd_e_(4:6)','zyx'));
xd_e =[xd_e_(1:3); EAAd_e(1:3)'*EAAd_e(4)];

    % Calculate the acceleration -  axang2rotm([d(4:6)./norm(d(4:6)) norm(d(4:6))]); 
xdd_e = transpose( diff([xd_e xd_e_old]')./dt );

%xdd_e = zeros(6,1);
%xdd_e(1:3) = transpose( diff([xd_e(1:3) xd_e_old(1:3)]')./dt );

%xdd_e(4:6) = (xd_e(4:6) - xd_e_old(4:6))/dt 

% vel_diff = cross(xd_e(4:6), xd_e_old(4:6));
% acc = norm(vel_diff)/dt
% if(acc < 1e-5)
%     xdd_e(4:6) = [0;0;0];
% else
%     xdd_e(4:6) = normalize(vel_diff)*acc;
% end

% acc_eaa = axang2rotm([vel_diff./norm(vel_diff) norm(vel_diff)]);
% 
% acc_dir = cross(xd_e(4:6), xd_e_old(4:6))./ 
% norm(cross(xd_e(4:6), xd_e_old(4:6)))
% 
% 
% cross(xd_e(4:6), xd_e_old(4:6))./dt;

FT_e = [R_ * robot.FTcp; R_ * robot.TTcp];

end
