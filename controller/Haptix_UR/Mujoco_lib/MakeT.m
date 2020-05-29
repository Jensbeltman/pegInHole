%==========================================================================
function T = MakeT(R,x)
%==========================================================================
% makes transformation matrix from rotation matrix (R) and position (x)
    if size(x,2) == 3
        x = x';
end
    T = [R x; 0 0 0 1];
end
