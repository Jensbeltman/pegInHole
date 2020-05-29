%==========================================================================
function [desired, tau, measured] = PDGravityPos(x_d)
%==========================================================================
% Performs an operational space PD control to the desired end-effector 
% position.  
%
% Parameters
%   x_d - desired position [x y z]

% Variables maintained in the function
%   g_q - gravitational compensation
%   x_e - end-effector position 
%   x_tilde - error term 
%   Ja - Analytical Jacobian
%   qdot - joint velocity
%==========================================================================

ur5 = importrobot('ur5_robot.urdf');    % For gravity computations
ur5.DataFormat = 'col';
 
Kp = [350 350 350]';                    % Proportional gain
Kd = [10 10 10]';                       % Derivative gain
g0 = [0 0 -9.81]';                      % Gravitational constant
m = [3.7 8.393 3.585 1 1.1 0.1]';       % Mass of the links 

tol = 0.01;                             % Tolerance on 1 cm

    % Initialize 
x_tilde = [100 100 100];                % Error
g_q = [0 0 0 0 0 0]';
count = 0;                              % Time counter
    % Values to plot
measured = [];
desired = [];
tau = [];

while norm(x_tilde) >= tol              % Until pos is within tolerance 
        % Current state of robot manipulator
    robot = RecMuJoCoData;
    q = robot.JntPos; 
    qdot = robot.JntVel;
    x_e = kinjac_UR5Tcp1(q);
    
        % Calculate the error term
    x_tilde = x_d - x_e;
        % Calculate the analytic Jacobian
    Ja = JacobianA(q);
    Ja_T = JacobianA(q)';
    
        % Calculate the geometric Jacobian from base to specified links
    J = {geometricJacobian(ur5, q, 'shoulder_link') geometricJacobian(ur5, q, 'upper_arm_link') ... 
            geometricJacobian(ur5, q, 'forearm_link') geometricJacobian(ur5, q, 'wrist_1_link') ...
            geometricJacobian(ur5, q, 'wrist_2_link') geometricJacobian(ur5, q, 'wrist_3_link') ...
            geometricJacobian(ur5, q, 'ee_link')};
    
        % Reset dE/dq   
    pE_pot = [0 0 0 0 0 0]';
     for qi=1:6 % For joint 1 to 6
         for li=1:6 % For link 1 to 6
                % Add effect from gravity on link(i) w.r.t current joint
             pE_pot(qi) = pE_pot(qi) - m(li)*g0'*J{li}(4:6,qi); 
         end
            % Add to gravity compensation vector
         g_q(qi) = pE_pot(qi);
     end
    
        % Compute the control signal
     u = g_q + Ja_T(:,1:3)*(Kp.*x_tilde) - Ja_T(:,1:3)*(Kd.*(Ja(1:3,:)*qdot));
     
        % Apply input signal to robot
     applied = mj_get_applied();
     applied.qfrc_applied;
     applied.qfrc_applied = u;
     mj_set_applied(applied);
    
        % Save values to plot 
     measured(:,end+1) = x_e(1:3);
     desired(:,end+1) = x_d(1:3);
     tau(:,end+1) = u;

        % Count up number of steps
     count = count + 1;  
end
disp(count);
end