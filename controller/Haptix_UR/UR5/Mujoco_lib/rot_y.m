function r = rot_y(t)
%#codegen

if isscalar(t)
    ct = cos(t);
    st = sin(t);
    r = [ct  0 st;
        0  1  0;
        -st  0 ct];
else
    error('Bad input dimensions. Input must be scalar');
end
