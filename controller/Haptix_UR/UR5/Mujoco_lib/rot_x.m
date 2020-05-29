function r = rot_x(t)
%#codegen

if isscalar(t)
    ct = cos(t);
    st = sin(t);
    r = [1   0   0
        0  ct -st
        0  st  ct];
else
    error('Bad input dimensions. Input must be scalar');
end