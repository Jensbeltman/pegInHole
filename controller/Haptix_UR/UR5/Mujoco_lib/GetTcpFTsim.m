function FT = GetTcpFTsim
robot = RecMuJoCoData;  
FT = [robot.FTcp',robot.TTcp'];
