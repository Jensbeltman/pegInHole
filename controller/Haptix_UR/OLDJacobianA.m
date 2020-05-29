function Ja = JacobianA(q)
% Function to calculate analytical Jacobian
% Ja: Analytical JAcobian given in euler angles 'zyz'
% q: Current joint positon 

    % Find the Geometric Jacobian
[P,R,J] = kinjac_UR5Tcp(q);

    % Convert to euler 'ZYZ' angles
Q = quaternion(R,'rotmat','point');
eulerAngles = euler(Q,'ZYZ','point');


phi = eulerAngles(1);
theta = eulerAngles(2);
psi = eulerAngles(3);

T_phi = [   0 -sin(phi) cos(phi)*sin(theta);
            0 cos(phi) sin(phi)*sin(theta);
            1 0 cos(theta)] ;

Transf = [eye(3,3), zeros(3,3); zeros(3,3), inv(T_phi)];
Ja = Transf*J;

end