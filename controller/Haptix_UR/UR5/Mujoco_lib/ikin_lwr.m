function qi = ikin_lwr(x,R,tcp,q)
Qr = quaternion(R);
[xi,Ri] = kinjac_lwr(q,tcp);
qi = q;
K = 0.5;
count = 0;
% first number is position precision and the second is orientation
% precision
while (norm(x-xi) > 0.0005) || (norm(R-Ri) > 0.01)
%while count < 100    
    [xi,Ri,Ji] = kinjac_lwr(qi,tcp); 
    %position controller
    xc = K*(x-xi);
    %orientation controller
    Qi = quaternion(Ri);
    Qc = Qr*inv(Qi);
    %    
    qdi = pinv(Ji)*K*[xc';real(Qc.v)'];
    %integration
    qi = qi + qdi;
    %check
    count = count + 1;
    if count > 100
        throw(MException('','ikin diverges'));
    end
%     if any(abs(qi) > 2*pi)
%         throw(MException('','ikin q > 2*pi'));
%     end
end;