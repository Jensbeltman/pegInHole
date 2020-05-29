%==========================================================================
function [Data] = JmoveM(q,t)
%==========================================================================
% join move from teh current joints to the (q) pose in (t) secconds
dt=1/100;
tn=0.0;
disp('jmove..')

%read robot data
Con = mj_get_control;
q0 = Con.ctrl(1:7)'; % from last position
if norm(q-q0) < 0.05
    disp('already there')
    return
end;
time =  dt:dt:t;
qi = jtraj(q0,q,time);
qdi = diff(qi)/dt;
qdi = [qdi;qdi(end,:)];
%execute
st = tic;

for i = 1:size(qi,1)
    Con.ctrl(1:7) = qi(i,:);
    Con.ctrl(8:14)= qdi(i,:);
    mj_set_control(Con);
    robot = RecMuJoCoData;
    SeDat(:,i) = robot.JntPos ;
    SeOri(:,i) = robot.CartOri;
    SeFTcp(:,i) = robot.FTcp;
        tn = tn+dt;
    if tn>toc(st)
        pause(tn-toc(st))
    end
end

 disp('done.');
end