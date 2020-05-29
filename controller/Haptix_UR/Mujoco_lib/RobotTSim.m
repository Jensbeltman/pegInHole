%==========================================================================
function T = RobotTSim(mode,tcp)
%==========================================================================
% returns current robot position in homogenous matrix
% mode c - commanded
% mode m - measured
% followed by
% mode b - base
% mode w - world
if nargin < 3
    tcp = [0 0 0];
end
switch lower(mode)
    case ('m')
        robot = RecMuJoCoData;
        x=robot.CartPos;
        R = quat2rotm(robot.CartOri');
        T = MakeT(R,x);
    case ('c')
        robot = RecMuJoCoData;
        q = robot.JntPos;
        [p,R] = kinjac_lwr(q,tcp);
        T = MakeT(R,p);
    otherwise
        throw(MException('','Unknown mode1 parameter'));
end
T = double(T);
end