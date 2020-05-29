function Gripper(state)


switch (state)
    case {'open'}
        Con = mj_get_control;
        Con.ctrl(15)=1;
        mj_set_control(Con);
        disp('Gripper Open')
        
    case {'close'}
        Con = mj_get_control;
        Con.ctrl(15)=0;
        mj_set_control(Con);
        disp('Gripper Close')
end

end