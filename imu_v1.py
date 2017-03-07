from sense_hat import SenseHat


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
#not sure how it indexes this
rate_gyro_x += gyro_raw[0] * S_cali
rate_gyro_y += gyro_raw[1] * S_cali
rate_gryo_z += gyro_raw[2] * S_cali

#Calculate over certain amount of time, get value into degrees:
gyro_x_angle += rate_gyro_x * dt
gyro_y_angle += rate_gyro_y * dt
gyro_z_angle += rate_gyro_z * dt


#Compare to sensehat polished values
gyro_only = sense.get_gyroscope()
print("p: {pitch}, r: {roll}, y: {yaw}".format(**gyro_only))
print("rawx:" + gyro_x_angle + "rawy:" + gyro_y_angle + "rawz:" + gyro_z_angle)


#Grabbing accel data
