% For this case, you just need to make sure that you are maintaining contact. 
% That means you don't really need stiffness (K=0), and then you want to push with a force 
% (F_target != 0). Then you would need to find a suitable inertia (M) and damping (D). 

function out = ImpedanceControl(desired_position, desired_acceleration, desired_force)
    % Crash if dt is too low
dt = 0.1;

    % Tolerance
tol_pos = [10^(-2); 10^(-2); 10^(-2)];
tol_acc = [10^(-2); 10^(-2); 10^(-2)];
tol_F   = [10^(-2); 10^(-2); 10^(-2)]; 
tol     = [tol_pos; tol_acc; tol_F];
    
K = 100; % Stiffness - Equal zero because we make sure we maintain contact
M = 0.200; % Inertia [kg] 
D = 500;  % Damping 

% Desired/target (3x1)
x_d = desired_position;      
xdd_d = desired_acceleration;
F_d = desired_force;

% Measured - Joint position, Joint velocity and TF at end effector
robot = RecMuJoCoData;
F_e = robot.FTcp
q_e = robot.JntPos;
qd_e = robot.JntVel;

% Measure the position, velocity and acceration at the end-effector. (3x1)
x_e   = kinjac_UR5Tcp(q_e);
xd_e  = OLDJacobianA(q_e) * qd_e;
xd_e  = xd_e(1:3);
xdd_e = [0;0;0];

% Calculate the errors vectors. (3x1)
x_err   = x_d - x_e;
xdd_err = xdd_d - xdd_e;
F_err   = F_d - F_e;

% Calculate the velocity given to the controller
xd_out = K*(x_err) ...
    + M*(xdd_err)...
    - D*xd_e...
    - F_err;
xd_out = xd_out/D;

    % Displace the end-effector in [x y z]. (1x3)
out = CmoveForUR5M(xd_out'.*dt ,dt);


% inc_q = qd*dt 
% q_new = q + inc_q


        % Repeat
% while (any( [x_e; xdd_e; F_e] > tol))
for i= 1:100
    if (all([x_err; xdd_err; F_err] < tol))
        display('Tol')
        return
    end
    %[x_err; xdd_err; F_err]
        % Save the old measurement to the acceleration can be derived
    xd_e_old = xd_e;

        % Measured - Joint position, Joint velocity and TF at end effector
    robot = RecMuJoCoData;
    F_e = robot.FTcp;
    q_e = robot.JntPos;
    qd_e = robot.JntVel;

    % Measure the position, velocity and acceration at the end-effector. (3x1)
    x_e   = kinjac_UR5Tcp(q_e);
    xd_e  = OLDJacobianA(q_e) * qd_e;
    xd_e  = xd_e(1:3);
    xdd_e = transpose( diff([xd_e_old xd_e]')./dt );

        % Calculate the errors vectors. (3x1)
    x_err   = x_d - x_e;
    xdd_err = xdd_d - xdd_e;
    F_err   = F_d - F_e;

    % Calculate the velocity given to the controller
    xd_out = K*(x_err) ...
        + M*(xdd_err)...
        - D*xd_e...
        - F_err;
    xd_out = xd_out/D;

        % Displace the end-effector in [x y z]. (1x3)
    out = CmoveForUR5M(xd_out'.*dt ,dt);
    
    xd_out'
    [x_err'
    xdd_err'
    xd_e'
    F_err']
    
end

end

