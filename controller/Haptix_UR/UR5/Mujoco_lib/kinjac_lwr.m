function [x,R,J]=kinjac_lwr(q,tcp)
%kinjac_lwr   Direct kinematics of KUKA LWR robot
%
% Usage:    [x,R]=kinjac_lwr(q,tcp)
%           [x,R,J]=kinjac_lwr(q,tcp)
%
% Input:    q   joint position (7 x 1)
%           tcp tool (3 x 1)
%
% Output:   x   task position (3 x 1)
%           R   rotational matrix (3 x 3)
%           J   Jacobian matrix (6 x n7)
%    

% Copyright (c) 2010 by IJS Bojan Nemec, Leon Zlajpah, Damir Omrcen
% Version: 1.0
%  
    L1 = 0.31;   % From base to first link 
    L2 = 0.4;    % Segment 1 
    L3 = 0.39;   % Segment 2 
    L4 = 0.078;  % From last segment to end-position 

% modification Pa10 --> LWR        
    q(1)=q(1)-pi;
    q(4)=-q(4);
    
    toolx=tcp(1);
    tooly=tcp(2);
    toolz=tcp(3);

    s1=sin(q(1));
    s2=sin(q(2));
    s3=sin(q(3));
    s4=sin(q(4));
    s5=sin(q(5));
    s6=sin(q(6));
    s7=sin(q(7));

    c1=cos(q(1));
    c2=cos(q(2));
    c3=cos(q(3));
    c4=cos(q(4));
    c5=cos(q(5));
    c6=cos(q(6));
    c7=cos(q(7));


% task position 
    x(1) = L3*s2*c4*c1-s3*c7*c5*c2*c1*tooly+L4*s2*c6*c4*c1+s2*c6*c4*c1*...
            toolz+L3*s4*c3*c2*c1+L2*s2*c1+c7*c6*c5*c4*c3*c2*c1*toolx-L4*...
            s6*s4*s2*c5*c1-s6*s4*c7*c3*c2*c1*toolx-s6*s2*c7*c4*c1*toolx+...
            L4*s6*c5*c4*c3*c2*c1+s6*c5*c4*c3*c2*c1*toolz+s5*s4*s2*c7*c1*...
            tooly-s5*s3*c7*c6*c2*c1*toolx-s5*c7*c4*c3*c2*c1*tooly-s4*s2*...
            c7*c6*c5*c1*toolx+L4*s4*c6*c3*c2*c1+s4*c6*c3*c2*c1*toolz+s7*...
            s6*s4*c3*c2*c1*tooly+s7*s6*s2*c4*c1*tooly+s7*s5*s4*s2*c1*...
            toolx+s7*s5*s3*c6*c2*c1*tooly-s7*s5*c4*c3*c2*c1*toolx+s7*s4*...
            s2*c6*c5*c1*tooly-s7*s3*c5*c2*c1*toolx-s7*c6*c5*c4*c3*c2*c1*...
            tooly-L4*s6*s5*s3*c2*c1-s6*s5*s3*c2*c1*toolz-s6*s4*s2*c5*c1*...
            toolz+(s5*s3*c7*c4*tooly-L3*s4*s3-s7*c5*c3*toolx-s5*c7*c6*c3*...
            toolx-s4*s3*c6*toolz-s3*c7*c6*c5*c4*toolx-s7*s6*s4*s3*tooly+...
            s7*s5*s3*c4*toolx-s6*s5*c3*toolz+s6*s4*s3*c7*toolx+s7*s3*c6*...
            c5*c4*tooly-L4*s6*s3*c5*c4-s6*s3*c5*c4*toolz-c7*c5*c3*tooly-...
            L4*s4*s3*c6+s7*s5*c6*c3*tooly-L4*s6*s5*c3)*s1;
    x(2) = (L4*s2*c6*c4+L3*s4*c3*c2+L2*s2+L3*s2*c4+s7*s4*s2*c6*c5*tooly-...
            s7*c6*c5*c4*c3*c2*tooly+s6*c5*c4*c3*c2*toolz+c7*c6*c5*c4*c3*...
            c2*toolx-s7*s3*c5*c2*toolx-s6*s4*c7*c3*c2*toolx-s5*s3*c7*c6*...
            c2*toolx-s6*s5*s3*c2*toolz-s6*s2*c7*c4*toolx-s6*s4*s2*c5*...
            toolz+L4*s6*c5*c4*c3*c2-s5*c7*c4*c3*c2*tooly+s5*s4*s2*c7*...
            tooly-s3*c7*c5*c2*tooly+s4*c6*c3*c2*toolz-s4*s2*c7*c6*c5*...
            toolx+s7*s6*s4*c3*c2*tooly-s7*s5*c4*c3*c2*toolx+s7*s6*s2*c4*...
            tooly+s7*s5*s4*s2*toolx+s7*s5*s3*c6*c2*tooly-L4*s6*s5*s3*c2+...
            L4*s4*c6*c3*c2+s2*c6*c4*toolz-L4*s6*s4*s2*c5)*s1+s6*s3*c5*...
            c4*c1*toolz+L3*s4*s3*c1+L4*s4*s3*c6*c1+s7*c5*c3*c1*toolx-...
            s7*s3*c6*c5*c4*c1*tooly-s7*s5*s3*c4*c1*toolx+s3*c7*c6*c5*c4*...
            c1*toolx-s7*s5*c6*c3*c1*tooly-s5*s3*c7*c4*c1*tooly-s6*s4*s3*...
            c7*c1*toolx+s7*s6*s4*s3*c1*tooly+L4*s6*s3*c5*c4*c1+L4*s6*s5*...
            c3*c1+s4*s3*c6*c1*toolz+s5*c7*c6*c3*c1*toolx+c7*c5*c3*c1*...
            tooly+s6*s5*c3*c1*toolz;
    x(3) = (s5*s3*c7*c6*toolx+s6*s4*c7*c3*toolx-L3*s4*c3+s5*c7*c4*c3*...
            tooly+s7*c6*c5*c4*c3*tooly-c7*c6*c5*c4*c3*toolx+s7*s5*c4*c3*...
            toolx+s7*s3*c5*toolx-s7*s6*s4*c3*tooly+s6*s5*s3*toolz-L4*s6*...
            c5*c4*c3-s6*c5*c4*c3*toolz+L4*s6*s5*s3+s3*c7*c5*tooly-L4*s4*...
            c6*c3-s4*c6*c3*toolz-s7*s5*s3*c6*tooly)*s2+L4*c6*c4*c2+L2*...
            c2+s7*s5*s4*c2*toolx-L4*s6*s4*c5*c2-s4*c7*c6*c5*c2*toolx+s5*...
            s4*c7*c2*tooly+L1-s6*s4*c5*c2*toolz+c6*c4*c2*toolz+s7*s6*c4*...
            c2*tooly-s6*c7*c4*c2*toolx+s7*s4*c6*c5*c2*tooly+L3*c4*c2;


% rotation matrix
  
    R(1,1) = s7*s5*s4*s2*c1+s7*s5*s3*s1*c4-s7*s5*c4*c3*c2*c1-s7*s3*c5*c2*...
            c1-s7*s1*c5*c3+s6*s4*s3*s1*c7-s6*s4*c7*c3*c2*c1-s6*s2*c7*c4*...
            c1-s5*s3*c7*c6*c2*c1-s5*s1*c7*c6*c3-s4*s2*c7*c6*c5*c1-s3*s1*...
            c7*c6*c5*c4+c7*c6*c5*c4*c3*c2*c1;
    R(2,1) = s7*s5*s4*s2*s1-s7*s5*s3*c4*c1-s7*s5*s1*c4*c3*c2-s7*s3*s1*c5*...
            c2+s7*c5*c3*c1-s6*s4*s3*c7*c1-s6*s4*s1*c7*c3*c2-s6*s2*s1*c7*...
            c4-s5*s3*s1*c7*c6*c2+s5*c7*c6*c3*c1-s4*s2*s1*c7*c6*c5+s3*c7*...
            c6*c5*c4*c1+s1*c7*c6*c5*c4*c3*c2;
    R(3,1) = s7*s5*s4*c2+s7*s5*s2*c4*c3+s7*s3*s2*c5+s6*s4*s2*c7*c3-s6*c7*...
            c4*c2+s5*s3*s2*c7*c6-s4*c7*c6*c5*c2-s2*c7*c6*c5*c4*c3;
    R(1,2) = -s7*s6*s4*s3*s1+s7*s6*s4*c3*c2*c1+s7*s6*s2*c4*c1+s7*s5*s3*...
            c6*c2*c1+s7*s5*s1*c6*c3+s7*s4*s2*c6*c5*c1+s7*s3*s1*c6*c5*c4-...
            s7*c6*c5*c4*c3*c2*c1+s5*s4*s2*c7*c1+s5*s3*s1*c7*c4-s5*c7*c4*...
            c3*c2*c1-s3*c7*c5*c2*c1-s1*c7*c5*c3;
    R(2,2) = s7*s6*s4*s3*c1+s7*s6*s4*s1*c3*c2+s7*s6*s2*s1*c4+s7*s5*s3*s1*...
            c6*c2-s7*s5*c6*c3*c1+s7*s4*s2*s1*c6*c5+-s7*s3*c6*c5*c4*c1-...
            s7*s1*c6*c5*c4*c3*c2+s5*s4*s2*s1*c7-s5*s3*c7*c4*c1-s5*s1*c7*...
            c4*c3*c2-s3*s1*c7*c5*c2+c7*c5*c3*c1;
    R(3,2) = -s7*s6*s4*s2*c3+s7*s6*c4*c2-s7*s5*s3*s2*c6+s7*s4*c6*c5*c2+...
            s7*s2*c6*c5*c4*c3+s5*s4*c7*c2+s5*s2*c7*c4*c3+s3*s2*c7*c5;
    R(1,3) = -s6*s5*s3*c2*c1-s6*s5*s1*c3-s6*s4*s2*c5*c1-s6*s3*s1*c5*c4+...
            s6*c5*c4*c3*c2*c1-s4*s3*s1*c6+s4*c6*c3*c2*c1+s2*c6*c4*c1;
    R(2,3) = -s6*s5*s3*s1*c2+s6*s5*c3*c1-s6*s4*s2*s1*c5+s6*s3*c5*c4*c1+...
            s6*s1*c5*c4*c3*c2+s4*s3*c6*c1+s4*s1*c6*c3*c2+s2*s1*c6*c4;
    R(3,3) = s6*s5*s3*s2-s6*s4*c5*c2-s6*s2*c5*c4*c3-s4*s2*c6*c3+c6*c4*c2;

    % modification Pa10 --> LWR
    R = R*[-1 0 0;0 -1 0; 0 0 1];
    
    if nargout>2
    % Jacobian - position part 
    J(1,1) =  (((((-s1*c2*c3-c1*s3)*c4+s1*s2*s4)*c5+(s1*c2*s3-c1*c3)*s5)*c6+(-(-s1*c2*c3-c1*s3)*s4+s1*s2*c4)*s6)*c7+(-((-s1*c2*c3-c1*s3)*c4+s1*s2*s4)*s5+(s1*c2*s3-c1*c3)*c5)*s7)*toolx+(-((((-s1*c2*c3-c1*s3)*c4+s1*s2*s4)*c5+(s1*c2*s3-c1*c3)*s5)*c6+(-(-s1*c2*c3-c1*s3)*s4+s1*s2*c4)*s6)*s7+(-((-s1*c2*c3-c1*s3)*c4+s1*s2*s4)*s5+(s1*c2*s3-c1*c3)*c5)*c7)*tooly+((((-s1*c2*c3-c1*s3)*c4+s1*s2*s4)*c5+(s1*c2*s3-c1*c3)*s5)*s6-(-(-s1*c2*c3-c1*s3)*s4+s1*s2*c4)*c6)*toolz+L4*(((-s1*c2*c3-c1*s3)*c4+s1*s2*s4)*c5+(s1*c2*s3-c1*c3)*s5)*s6-L4*(-(-s1*c2*c3-c1*s3)*s4+s1*s2*c4)*c6+L3*(-s1*c2*c3-c1*s3)*s4-L3*s1*s2*c4-L2*s1*s2; 
    J(1,2) = ((((-c1*s2*c3*c4-c1*c2*s4)*c5+c1*s2*s3*s5)*c6+(c1*s2*c3*s4-c1*c2*c4)*s6)*c7+(-(-c1*s2*c3*c4-c1*c2*s4)*s5+c1*s2*s3*c5)*s7)*toolx+(-(((-c1*s2*c3*c4-c1*c2*s4)*c5+c1*s2*s3*s5)*c6+(c1*s2*c3*s4-c1*c2*c4)*s6)*s7+(-(-c1*s2*c3*c4-c1*c2*s4)*s5+c1*s2*s3*c5)*c7)*tooly+(((-c1*s2*c3*c4-c1*c2*s4)*c5+c1*s2*s3*s5)*s6-(c1*s2*c3*s4-c1*c2*c4)*c6)*toolz+L4*((-c1*s2*c3*c4-c1*c2*s4)*c5+c1*s2*s3*s5)*s6-L4*(c1*s2*c3*s4-c1*c2*c4)*c6-L3*c1*s2*c3*s4+L3*c1*c2*c4+L2*c1*c2;
    J(1,3) = ((((-c1*c2*s3-s1*c3)*c4*c5+(-c1*c2*c3+s1*s3)*s5)*c6-(-c1*c2*s3-s1*c3)*s4*s6)*c7+(-(-c1*c2*s3-s1*c3)*c4*s5+(-c1*c2*c3+s1*s3)*c5)*s7)*toolx+(-(((-c1*c2*s3-s1*c3)*c4*c5+(-c1*c2*c3+s1*s3)*s5)*c6-(-c1*c2*s3-s1*c3)*s4*s6)*s7+(-(-c1*c2*s3-s1*c3)*c4*s5+(-c1*c2*c3+s1*s3)*c5)*c7)*tooly+(((-c1*c2*s3-s1*c3)*c4*c5+(-c1*c2*c3+s1*s3)*s5)*s6+(-c1*c2*s3-s1*c3)*s4*c6)*toolz+L4*((-c1*c2*s3-s1*c3)*c4*c5+(-c1*c2*c3+s1*s3)*s5)*s6+L4*(-c1*c2*s3-s1*c3)*s4*c6+L3*(-c1*c2*s3-s1*c3)*s4;
    J(1,4) = (((-(c1*c2*c3-s1*s3)*s4-c1*s2*c4)*c5*c6+(-(c1*c2*c3-s1*s3)*c4+c1*s2*s4)*s6)*c7-(-(c1*c2*c3-s1*s3)*s4-c1*s2*c4)*s5*s7)*toolx+(-((-(c1*c2*c3-s1*s3)*s4-c1*s2*c4)*c5*c6+(-(c1*c2*c3-s1*s3)*c4+c1*s2*s4)*s6)*s7-(-(c1*c2*c3-s1*s3)*s4-c1*s2*c4)*s5*c7)*tooly+((-(c1*c2*c3-s1*s3)*s4-c1*s2*c4)*c5*s6-(-(c1*c2*c3-s1*s3)*c4+c1*s2*s4)*c6)*toolz+L4*(-(c1*c2*c3-s1*s3)*s4-c1*s2*c4)*c5*s6-L4*(-(c1*c2*c3-s1*s3)*c4+c1*s2*s4)*c6+L3*(c1*c2*c3-s1*s3)*c4-L3*c1*s2*s4;
    J(1,5) = ((-((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*s5+(-c1*c2*s3-s1*c3)*c5)*c6*c7+(-((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*c5-(-c1*c2*s3-s1*c3)*s5)*s7)*toolx+(-(-((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*s5+(-c1*c2*s3-s1*c3)*c5)*c6*s7+(-((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*c5-(-c1*c2*s3-s1*c3)*s5)*c7)*tooly+(-((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*s5+(-c1*c2*s3-s1*c3)*c5)*s6*toolz+L4*(-((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*s5+(-c1*c2*s3-s1*c3)*c5)*s6;
    J(1,6) = (-(((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*c5+(-c1*c2*s3-s1*c3)*s5)*s6+(-(c1*c2*c3-s1*s3)*s4-c1*s2*c4)*c6)*c7*toolx-(-(((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*c5+(-c1*c2*s3-s1*c3)*s5)*s6+(-(c1*c2*c3-s1*s3)*s4-c1*s2*c4)*c6)*s7*tooly+((((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*c5+(-c1*c2*s3-s1*c3)*s5)*c6+(-(c1*c2*c3-s1*s3)*s4-c1*s2*c4)*s6)*toolz+L4*(((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*c5+(-c1*c2*s3-s1*c3)*s5)*c6+L4*(-(c1*c2*c3-s1*s3)*s4-c1*s2*c4)*s6;
    J(1,7) = (-((((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*c5+(-c1*c2*s3-s1*c3)*s5)*c6+(-(c1*c2*c3-s1*s3)*s4-c1*s2*c4)*s6)*s7+(-((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*s5+(-c1*c2*s3-s1*c3)*c5)*c7)*toolx+(-((((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*c5+(-c1*c2*s3-s1*c3)*s5)*c6+(-(c1*c2*c3-s1*s3)*s4-c1*s2*c4)*s6)*c7-(-((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*s5+(-c1*c2*s3-s1*c3)*c5)*s7)*tooly;
    J(2,1) = (((((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*c5+(-c1*c2*s3-s1*c3)*s5)*c6+(-(c1*c2*c3-s1*s3)*s4-c1*s2*c4)*s6)*c7+(-((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*s5+(-c1*c2*s3-s1*c3)*c5)*s7)*toolx+(-((((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*c5+(-c1*c2*s3-s1*c3)*s5)*c6+(-(c1*c2*c3-s1*s3)*s4-c1*s2*c4)*s6)*s7+(-((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*s5+(-c1*c2*s3-s1*c3)*c5)*c7)*tooly+((((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*c5+(-c1*c2*s3-s1*c3)*s5)*s6-(-(c1*c2*c3-s1*s3)*s4-c1*s2*c4)*c6)*toolz+L4*(((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*c5+(-c1*c2*s3-s1*c3)*s5)*s6-L4*(-(c1*c2*c3-s1*s3)*s4-c1*s2*c4)*c6+L3*(c1*c2*c3-s1*s3)*s4+L3*c1*s2*c4+L2*c1*s2;
    J(2,2) = ((((-s1*s2*c3*c4-s1*c2*s4)*c5+s1*s2*s3*s5)*c6+(s1*s2*c3*s4-s1*c2*c4)*s6)*c7+(-(-s1*s2*c3*c4-s1*c2*s4)*s5+s1*s2*s3*c5)*s7)*toolx+(-(((-s1*s2*c3*c4-s1*c2*s4)*c5+s1*s2*s3*s5)*c6+(s1*s2*c3*s4-s1*c2*c4)*s6)*s7+(-(-s1*s2*c3*c4-s1*c2*s4)*s5+s1*s2*s3*c5)*c7)*tooly+(((-s1*s2*c3*c4-s1*c2*s4)*c5+s1*s2*s3*s5)*s6-(s1*s2*c3*s4-s1*c2*c4)*c6)*toolz+L4*((-s1*s2*c3*c4-s1*c2*s4)*c5+s1*s2*s3*s5)*s6-L4*(s1*s2*c3*s4-s1*c2*c4)*c6-L3*s1*s2*c3*s4+L3*s1*c2*c4+L2*s1*c2;
    J(2,3) = ((((-s1*c2*s3+c1*c3)*c4*c5+(-s1*c2*c3-c1*s3)*s5)*c6-(-s1*c2*s3+c1*c3)*s4*s6)*c7+(-(-s1*c2*s3+c1*c3)*c4*s5+(-s1*c2*c3-c1*s3)*c5)*s7)*toolx+(-(((-s1*c2*s3+c1*c3)*c4*c5+(-s1*c2*c3-c1*s3)*s5)*c6-(-s1*c2*s3+c1*c3)*s4*s6)*s7+(-(-s1*c2*s3+c1*c3)*c4*s5+(-s1*c2*c3-c1*s3)*c5)*c7)*tooly+(((-s1*c2*s3+c1*c3)*c4*c5+(-s1*c2*c3-c1*s3)*s5)*s6+(-s1*c2*s3+c1*c3)*s4*c6)*toolz+L4*((-s1*c2*s3+c1*c3)*c4*c5+(-s1*c2*c3-c1*s3)*s5)*s6+L4*(-s1*c2*s3+c1*c3)*s4*c6+L3*(-s1*c2*s3+c1*c3)*s4;
    J(2,4) = (((-(s1*c2*c3+c1*s3)*s4-s1*s2*c4)*c5*c6+(-(s1*c2*c3+c1*s3)*c4+s1*s2*s4)*s6)*c7-(-(s1*c2*c3+c1*s3)*s4-s1*s2*c4)*s5*s7)*toolx+(-((-(s1*c2*c3+c1*s3)*s4-s1*s2*c4)*c5*c6+(-(s1*c2*c3+c1*s3)*c4+s1*s2*s4)*s6)*s7-(-(s1*c2*c3+c1*s3)*s4-s1*s2*c4)*s5*c7)*tooly+((-(s1*c2*c3+c1*s3)*s4-s1*s2*c4)*c5*s6-(-(s1*c2*c3+c1*s3)*c4+s1*s2*s4)*c6)*toolz+L4*(-(s1*c2*c3+c1*s3)*s4-s1*s2*c4)*c5*s6-L4*(-(s1*c2*c3+c1*s3)*c4+s1*s2*s4)*c6+L3*(s1*c2*c3+c1*s3)*c4-L3*s1*s2*s4;
    J(2,5) = ((-((s1*c2*c3+c1*s3)*c4-s1*s2*s4)*s5+(-s1*c2*s3+c1*c3)*c5)*c6*c7+(-((s1*c2*c3+c1*s3)*c4-s1*s2*s4)*c5-(-s1*c2*s3+c1*c3)*s5)*s7)*toolx+(-(-((s1*c2*c3+c1*s3)*c4-s1*s2*s4)*s5+(-s1*c2*s3+c1*c3)*c5)*c6*s7+(-((s1*c2*c3+c1*s3)*c4-s1*s2*s4)*c5-(-s1*c2*s3+c1*c3)*s5)*c7)*tooly+(-((s1*c2*c3+c1*s3)*c4-s1*s2*s4)*s5+(-s1*c2*s3+c1*c3)*c5)*s6*toolz+L4*(-((s1*c2*c3+c1*s3)*c4-s1*s2*s4)*s5+(-s1*c2*s3+c1*c3)*c5)*s6;
    J(2,6) = (-(((s1*c2*c3+c1*s3)*c4-s1*s2*s4)*c5+(-s1*c2*s3+c1*c3)*s5)*s6+(-(s1*c2*c3+c1*s3)*s4-s1*s2*c4)*c6)*c7*toolx-(-(((s1*c2*c3+c1*s3)*c4-s1*s2*s4)*c5+(-s1*c2*s3+c1*c3)*s5)*s6+(-(s1*c2*c3+c1*s3)*s4-s1*s2*c4)*c6)*s7*tooly+((((s1*c2*c3+c1*s3)*c4-s1*s2*s4)*c5+(-s1*c2*s3+c1*c3)*s5)*c6+(-(s1*c2*c3+c1*s3)*s4-s1*s2*c4)*s6)*toolz+L4*(((s1*c2*c3+c1*s3)*c4-s1*s2*s4)*c5+(-s1*c2*s3+c1*c3)*s5)*c6+L4*(-(s1*c2*c3+c1*s3)*s4-s1*s2*c4)*s6;
    J(2,7) = (-((((s1*c2*c3+c1*s3)*c4-s1*s2*s4)*c5+(-s1*c2*s3+c1*c3)*s5)*c6+(-(s1*c2*c3+c1*s3)*s4-s1*s2*c4)*s6)*s7+(-((s1*c2*c3+c1*s3)*c4-s1*s2*s4)*s5+(-s1*c2*s3+c1*c3)*c5)*c7)*toolx+(-((((s1*c2*c3+c1*s3)*c4-s1*s2*s4)*c5+(-s1*c2*s3+c1*c3)*s5)*c6+(-(s1*c2*c3+c1*s3)*s4-s1*s2*c4)*s6)*c7-(-((s1*c2*c3+c1*s3)*c4-s1*s2*s4)*s5+(-s1*c2*s3+c1*c3)*c5)*s7)*tooly;
    J(3,1) = 0;
    J(3,2) = ((((-c2*c3*c4+s2*s4)*c5+c2*s3*s5)*c6+(c2*c3*s4+s2*c4)*s6)*c7+(-(-c2*c3*c4+s2*s4)*s5+c2*s3*c5)*s7)*toolx+(-(((-c2*c3*c4+s2*s4)*c5+c2*s3*s5)*c6+(c2*c3*s4+s2*c4)*s6)*s7+(-(-c2*c3*c4+s2*s4)*s5+c2*s3*c5)*c7)*tooly+(((-c2*c3*c4+s2*s4)*c5+c2*s3*s5)*s6-(c2*c3*s4+s2*c4)*c6)*toolz+L4*((-c2*c3*c4+s2*s4)*c5+c2*s3*s5)*s6-L4*(c2*c3*s4+s2*c4)*c6-L3*c2*c3*s4-L3*s2*c4-L2*s2;
    J(3,3) = (((s2*s3*c4*c5+s2*c3*s5)*c6-s2*s3*s4*s6)*c7+(-s2*s3*c4*s5+s2*c3*c5)*s7)*toolx+(-((s2*s3*c4*c5+s2*c3*s5)*c6-s2*s3*s4*s6)*s7+(-s2*s3*c4*s5+s2*c3*c5)*c7)*tooly+((s2*s3*c4*c5+s2*c3*s5)*s6+s2*s3*s4*c6)*toolz+L4*(s2*s3*c4*c5+s2*c3*s5)*s6+L4*s2*s3*s4*c6+L3*s2*s3*s4;
    J(3,4) = (((s2*c3*s4-c2*c4)*c5*c6+(s2*c3*c4+c2*s4)*s6)*c7-(s2*c3*s4-c2*c4)*s5*s7)*toolx+(-((s2*c3*s4-c2*c4)*c5*c6+(s2*c3*c4+c2*s4)*s6)*s7-(s2*c3*s4-c2*c4)*s5*c7)*tooly+((s2*c3*s4-c2*c4)*c5*s6-(s2*c3*c4+c2*s4)*c6)*toolz+L4*(s2*c3*s4-c2*c4)*c5*s6-L4*(s2*c3*c4+c2*s4)*c6-L3*s2*c3*c4-L3*c2*s4;
    J(3,5) = ((-(-s2*c3*c4-c2*s4)*s5+s2*s3*c5)*c6*c7+(-(-s2*c3*c4-c2*s4)*c5-s2*s3*s5)*s7)*toolx+(-(-(-s2*c3*c4-c2*s4)*s5+s2*s3*c5)*c6*s7+(-(-s2*c3*c4-c2*s4)*c5-s2*s3*s5)*c7)*tooly+(-(-s2*c3*c4-c2*s4)*s5+s2*s3*c5)*s6*toolz+L4*(-(-s2*c3*c4-c2*s4)*s5+s2*s3*c5)*s6;
    J(3,6) = (-((-s2*c3*c4-c2*s4)*c5+s2*s3*s5)*s6+(s2*c3*s4-c2*c4)*c6)*c7*toolx-(-((-s2*c3*c4-c2*s4)*c5+s2*s3*s5)*s6+(s2*c3*s4-c2*c4)*c6)*s7*tooly+(((-s2*c3*c4-c2*s4)*c5+s2*s3*s5)*c6+(s2*c3*s4-c2*c4)*s6)*toolz+L4*((-s2*c3*c4-c2*s4)*c5+s2*s3*s5)*c6+L4*(s2*c3*s4-c2*c4)*s6;
    J(3,7) = (-(((-s2*c3*c4-c2*s4)*c5+s2*s3*s5)*c6+(s2*c3*s4-c2*c4)*s6)*s7+(-(-s2*c3*c4-c2*s4)*s5+s2*s3*c5)*c7)*toolx+(-(((-s2*c3*c4-c2*s4)*c5+s2*s3*s5)*c6+(s2*c3*s4-c2*c4)*s6)*c7-(-(-s2*c3*c4-c2*s4)*s5+s2*s3*c5)*s7)*tooly;

% Jacobian - orientation part
    J(4,1) = 0;
    J(4,2) = -s1;
    J(4,3) = c1*s2;
    J(4,4) = -c1*c2*s3-s1*c3;
    J(4,5) = (c1*c2*c3-s1*s3)*s4+c1*s2*c4;
    J(4,6) = -((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*s5+(-c1*c2*s3-s1*c3)*c5;
    J(4,7) = (((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*c5+(-c1*c2*s3-s1*c3)*s5)*s6+((c1*c2*c3-s1*s3)*s4+c1*s2*c4)*c6;
    J(5,1) = 0;
    J(5,2) = c1;
    J(5,3) = s1*s2;
    J(5,4) = -s1*c2*s3+c1*c3;
    J(5,5) = (s1*c2*c3+c1*s3)*s4+s1*s2*c4;
    J(5,6) = -((s1*c2*c3+c1*s3)*c4-s1*s2*s4)*s5+(-s1*c2*s3+c1*c3)*c5;
    J(5,7) = (((s1*c2*c3+c1*s3)*c4-s1*s2*s4)*c5+(-s1*c2*s3+c1*c3)*s5)*s6+((s1*c2*c3+c1*s3)*s4+s1*s2*c4)*c6;
    J(6,1) = 1;
    J(6,2) = 0;
    J(6,3) = c2;
    J(6,4) = s2*s3;
    J(6,5) = -s2*c3*s4+c2*c4;
    J(6,6) = -(-s2*c3*c4-c2*s4)*s5+s2*s3*c5;
    J(6,7) = ((-s2*c3*c4-c2*s4)*c5+s2*s3*s5)*s6+(-s2*c3*s4+c2*c4)*c6;

% modification Pa10 --> LWR
    J(:,4) = -J(:,4);
    end
