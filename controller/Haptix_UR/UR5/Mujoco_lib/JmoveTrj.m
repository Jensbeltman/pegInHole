%==========================================================================
function JmoveTrj(qi)
%==========================================================================
% join move from teh current joints to the (q) pose in (t) secconds
global robot
disp('jmove trj..')
qi=qi;
dt=1/100;
%read robot data
Con = mj_get_control;
%prepare trajectory
qdi = diff(qi)/dt;
[B,A] = butter(2,0.3);
qdif = filtfilt(B,A,qdi);
% be sure to end with 0 velocity
qdif = [qdif; zeros(1,7)];
tn=0;
st = tic;
for i = 1:length(qi)
    Con.ctrl(1:7) = qi(i,:);
    Con.ctrl(8:14)= qdif(i,:);
    mj_set_control(Con);
    robot = RecMuJoCoData;
    tn = tn+dt;
    if tn>toc(st)
        pause(tn-toc(st))
    end
    
end
disp('done.');
end