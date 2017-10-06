#!/usr/bin/env python 
import rospy
import math

from time import time, sleep
import sensor_msgs.msg
from sensor_msgs.msg import Imu
import geometry_msgs.msg
from sense_hat import SenseHat

sense = SenseHat()
gtoaccel = 9.81
imu = sensor_msgs.msg.Imu()
rospy.init_node("imu_node")
imu_pub = rospy.Publisher('Imu/imuTopic', Imu ,queue_size=500)
rate = rospy.Rate(1000)
sense.set_imu_config(False, True, True)

#comment



def imustream():
    
    #Need to find covariance values through testing. So blank right now
    #Subscriber will need to check to see: if element 0 is -1, then disregard this matrix

##    imuMsg.orientation_covariance = [-1 , 0 , 0,
##    0, 0, 0,
##    0, 0, 0]
##    imuMsg.angular_velocity_covariance = [-1, 0 , 0,
##    0 , 0, 0,
##    0 , 0 , 0]
##    imuMsg.linear_acceleration_covariance = [-1 , 0 , 0,
##    0 , 0, 0,
##    0 , 0 , 0]


    #rospy.sleep(5) # Sleep for 8 seconds to wait for the board to boot then only write command.

    #loop and stream IMU (gyro and accel) data
    while not rospy.is_shutdown():
        o_deg = sense.get_orientation_degrees()
        print(o_deg)
        sense.show_message("Streaming data")
        gyro_raw = sense.get_gyroscope_raw()
        #values from gyro_raw already are in floats, this is publishing live as well
        imu.angular_velocity.x = float(gyro_raw["x"]) #gyro
        imu.angular_velocity.y = float(gyro_raw["y"])
        imu.angular_velocity.z = float(gyro_raw["z"])


        #values from accel (convert Gs to m/s^2, keep as floats)
        accel_raw = sense.get_accelerometer_raw()

        imu.linear_acceleration.x = float((accel_raw["x"] * gtoaccel)) # tripe axis accelerator meter
        imu.linear_acceleration.y = float((accel_raw["y"] * gtoaccel))
        imu.linear_acceleration.z = float((accel_raw["z"] * gtoaccel))


    

        


        #publish message with timestamp
        imu.header.stamp= rospy.Time.now()
        imu.header.frame_id = 'base_link'
        imu_pub.publish(imu)
        rate.sleep()




if __name__ == '__main__':
    try:
        imustream()
    except rospy.ROSInterruptException:
        pass

