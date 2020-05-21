# lanpodo2ros.h
int32       robot_state
int32       step_phase

float64[3]  pel_pos_est
float64[4]  pel_quaternion
float64[31] joint_reference         #RHY=0, RHR, RHP, RKN, RAP, RAR, LHY, LHR, LHP, LKN, LAP, LAR, WST=24
float64[31] joint_encoder

float64[12] ft_sensor               #RMx, RMy, RMz, RFx, RFy, RFz, LMx, LMy, LMz, LFx, LFy, LFz
float64[9]  imu_sensor              #Roll, Pitch, Yaw, dRoll, dPitch, dYaw, ddRoll, ddPitch, ddYaw
