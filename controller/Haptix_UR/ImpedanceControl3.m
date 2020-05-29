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
dt = 1/500;

    % Tolerance
tol_pos = [10^(-3); 10^(-3); 10^(-3); 10^(-2); 10^(-2); 10^(-2)];
tol_acc = [10^(-2); 10^(-2); 10^(-2)];
tol_F   = [10^(-2); 10^(-2); 10^(-2); 10^(-2); 10^(-2); 10^(-2)]; 
tol     = [tol_pos; tol_acc; tol_F];

    % Stiffness vector
K_pos = ones(3,1) .* [2000; 2000; 2000];
K_ori = ones(3,1) * 1000;
K = [K_pos; K_ori]; 
    % Inertia vector
M_pos = ones(3,1) * 5;
M_ori = ones(3,1) * 0.5;
M = [M_pos; M_ori]; % Inertia [kg] 
    % Damping vector
D_pos = ones(3,1) * 1000;
D_ori = ones(3,1) * 1000;
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


while (~all( abs(x_d - x_e) < tol_pos) || ~all( abs(F_d - FT_e) < tol_F ))
%while (1)
    

%     st_ = tic;
        % Save the old measurement to the acceleration can be derived
    [x_e, xd_e, xdd_e, FT_e] = update_state(xd_e, dt);
    
            % Forces
    forces = FT_e - desired_force;
    
        %Position
    pos_stiffness = K(1:3).* (x_d(1:3) - x_e(1:3));
    pos_inertia = M(1:3).* (xdd_d(1:3) - xdd_e(1:3));
    pos_damping = D(1:3).*(- xd_e(1:3));
    xd_pos_target = (pos_stiffness + pos_inertia + pos_damping - forces(1:3))./ D(1:3);

        % Orientation
    rot_stiffness = K(4:6).* cross(x_d(4:6), x_e(4:6));
    rot_inertia = M(4:6).* cross(xdd_d(4:6), xdd_e(4:6));
    rot_damping = D(4:6).* cross(zeros(3,1), xd_e(4:6));
    xd_rot_target = (rot_stiffness + rot_inertia + rot_damping + forces(4:6))./ D(4:6);

    
    xd_target = [xd_pos_target; xd_rot_target];
    
   
        % Displace the end-effector in [x y z eaa]
        
%     elaps = toc(st_);
%     dt - elaps
    Data = CmoveForUR5M_ori(xd_target' ,dt);
    
    
%     tn = tn+dt
%     if tn>toc(st)
%         disp('__')
%         pause(tn-toc(st))
%     end
    

    
   
%     [FT_e'
%     forces'
%     xd_target']    

%     disp('__')
%     cross(x_d(4:6), x_e(4:6))'
%     rad2deg( asin(norm(cross(x_d(4:6), x_e(4:6)))/(norm(x_d(4:6))* norm(x_e(4:6)))) )
     xd_target'

    t.Ftcp_C = Data.Ftcp;
    t.Ftcp_I = FT_e(1:3); 
    t.pos = x_e;
    
    out(i) = {t};

    
%     if(x_d - x_e < tol_pos)
%         display('Tol')
%         %return 
%     end
if i == 500
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
R = rotm2axang(R_); R = R(1:3)*R(4);
x_e = [x_e; R'];
    % Get the velocity an convert it to EAA 
xd_e  = JacobianA(q_e) * qd_e;  % Euler 'zyx'
%EAAd_e = rotm2axang(eul2rotm(xd_e(4:6)','zyx'));
%xd_e =[xd_e(1:3); EAAd_e(1:3)'*EAAd_e(4)];
xdd_e = transpose( diff([xd_e xd_e_old]')./dt );



FT_e = [R_ * robot.FTcp; R_ * robot.TTcp];

end
