function T_i = cLinearMove(T_start, T_end, t, dt)
% Given a start pose and a end pose, calculate the constant velocity linear 
% interpolatuon in tool space consist of the path linear movement
% P(t) = p^(i-1) + (t - t_(i-1))/(t_i - t_(i-1)) (p^i - p^(i-1))
% R(t) = Slerp(quat(R^(i-1), quat(R^i, t)

% T_start: The initial pose [homegeneous transpose]
% T_end  : The end pose [homegeneous transpose]
% t      : The time duration [sec]
% dt     : The time step
% T_i    : Output containing a series of poses contained in a cell T_i = {}
%          named T_i.pose 


    %% Split the poses into position and Orientation
R_start = T_start(1:3, 1:3);
p_start = T_start(1:3,4);

R_end = T_end(1:3, 1:3);
p_end = T_end(1:3,4);

steps = t/dt;
T_i ={};
    % start from i=1 => we dont need to move to current position
for i = 1:steps
        % Linear interpolate between positions
    p_i = p_start + (i*dt/t)*(p_end - p_start);
    
        % Linear interpolate between orientations        
    Q_start = quaternion(R_start,'rotmat','point');
    Q_end = quaternion(R_end,'rotmat','point');
         
    interpolationCoefficient = i/steps;    % Slep need values [0;1]
    R_i = slerp(Q_start,Q_end,interpolationCoefficient);
    R_i = quat2rotm(R_i);
        % display in degrees 
    %averageRotation = eulerd(R_i,'ZYX','frame')
     
        % Save the data to the cell
    T_i(i).pose =  [R_i p_i; 0 0 0 1];
end

end