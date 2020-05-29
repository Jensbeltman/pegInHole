function robot = RecMuJoCoData
sensor=mj_get_sensor;
robot.JntPos = sensor.sensordata(1:6);
robot.JntVel = sensor.sensordata(7:12);
robot.CartPos = sensor.sensordata(13:15);
robot.CartOri = sensor.sensordata(16:19);
robot.FTcp = sensor.sensordata(20:22)*10;
robot.TTcp = sensor.sensordata(23:25);
% robot.Gripper = sensor.sensordata(28);
end