from sense_hat import SenseHat
import math


sense = SenseHat()
#disable mag, enable gyro and accel
sense.set_imu_config(False, True, True)

#Start with preparing gyroscope data

#Sensitivity Calibration Value, currently assuming 2000 DPS
S_cali = 0.07
#Loop period
dt = 0.02

gyro_raw = sense.get_gyroscope_raw()

#Convert to DPS (Degrees per second) account for rate of change for the kalman filter
#not sure how it indexes this...I am assuming string indeces
rate_gyro_x += gyro_raw["x"] * S_cali
rate_gyro_y += gyro_raw["y"] * S_cali
rate_gryo_z += gyro_raw["z"] * S_cali

#Calculate over certain amount of time, get value into degrees:
gyro_x_angle += rate_gyro_x * dt
gyro_y_angle += rate_gyro_y * dt
gyro_z_angle += rate_gyro_z * dt


#Compare to sensehat polished values
gyro_only = sense.get_gyroscope()
print("p: {pitch}, r: {roll}, y: {yaw}".format(**gyro_only))
print("rawx:" + gyro_x_angle + "rawy:" + gyro_y_angle + "rawz:" + gyro_z_angle)


#Grabbing accel data
M_PI = 3.14159265358979323846
RAD_TO_DEG = 57.29578 
accel_raw = sense.get_accelerometer_raw()
AccXAngle = (atan2(accel_raw["y"],accel_raw["z"]+M_PI) * RAD_TO_DEG)
AccYAngle = (atan2(accel_raw["z"],accel_raw["x"]+M_PI) * RAD_TO_DEG)

#Print polished results
accel_only = sense.get_accelerometer()
print("p: {pitch}, r: {roll}, y: {yaw}".format(**accel_only))
print("rawx:" + AccXAngle + "rawy:" + AccYAngle)
