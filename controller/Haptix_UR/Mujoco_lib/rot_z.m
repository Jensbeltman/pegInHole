function r = rot_z(t)
%#codegen

if isscalar(t)
    ct = cos(t);
    st = sin(t);
    r = [ct -st 0;
        st  ct 0;
        0	0	1];
else
    error('Bad input dimensions. Input must be scalar');
end
