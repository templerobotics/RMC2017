from sense_hat import SenseHat
import math
from time import sleep


sense = SenseHat()
#disable mag, enable gyro and accel
sense.set_imu_config(False, True, True)

#Start with preparing gyroscope data

#Sensitivity Calibration Value, currently assuming 2000 DPS
S_cali = 0.07
#Loop period
dt = 0.02


#raw data outputted as radians per second

while True:
    gyro_raw = sense.get_gyroscope_raw()
    #Convert to DPS (Degrees per second) account for rate of change for the kalman filter
    rate_gyro_x = gyro_raw["x"] * 57.2958
    rate_gyro_y = gyro_raw["y"] * 57.2958
    rate_gyro_z = gyro_raw["z"] * 57.2958

    #Calculate over certain amount of time, get value into degrees:
    gyro_x_angle = rate_gyro_x * dt
    gyro_y_angle = rate_gyro_y * dt
    gyro_z_angle = rate_gyro_z * dt


    #Compare to sensehat polished values
    gyro_only = sense.get_gyroscope()
    print("Gyro Results")
    print("p: {pitch}, r: {roll}, y: {yaw}".format(**gyro_only))
    print(gyro_x_angle,gyro_y_angle,gyro_z_angle)
    print(rate_gyro_x,rate_gyro_y,rate_gyro_z)

    sleep(2)
    



###Grabbing accel data
##M_PI = 3.14159265358979323846
##RAD_TO_DEG = 57.29578 
##accel_raw = sense.get_accelerometer_raw()
##AccXAngle = (atan2(accel_raw["y"],accel_raw["z"]+M_PI) * RAD_TO_DEG)
##AccYAngle = (atan2(accel_raw["z"],accel_raw["x"]+M_PI) * RAD_TO_DEG)
##
###Print polished result
##accel_only = sense.get_accelerometer()
##print("Accel Results")
##print("p: {pitch}, r: {roll}, y: {yaw}".format(**accel_only))
##print("rawx:" + AccXAngle + "rawy:" + AccYAngle)
##
##
###from here apply filter (And we will get final X and Y angles)
