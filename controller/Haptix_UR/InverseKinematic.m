% This function finds the inverse kindematic of the ur5 robot, given the
% position and orientation specified in the robot base frame 

function q = InverseKinematic(pos_, ori_, initialguess_)
robot_ = importrobot('./UR5/ur5_robot.urdf');
robot_.DataFormat = 'column';


    % Rotation to convert between the robotics toolbox and the simulator
R_w_b = rotz(pi);
R_e_ee= quat2rotm([0.5 -0.5 0.5 -0.5]);

tform_ = trvec2tform(pos_*R_w_b)*rotm2tform(R_w_b*(axang2rotm([0 1 0 pi]))*ori_*inv(R_e_ee));


    % Do the inverse kinematics
ik_ = inverseKinematics('RigidBodyTree',robot_);
weights_ = [0.1 0.1 0.1 0.2 0.2 0.2];

%[configSoln_,solnInfo] = ik_('wrist_3_link',tform_, weights_,initialguess_);
[configSoln_,solnInfo] = ik_('ee_link',tform_, weights_,initialguess_);
q = configSoln_;
end