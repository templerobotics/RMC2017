import roslib; roslib.load_manifest('razor_imu_9dof')
import rospy
import math

from time import time, sleep
from sensor_msgs.msg import Imu
from sense_hat import SenseHat
import tf

gtoaccel = 9.81

rospy.init_node("node")
imu_pub = rospy.Publisher('Imu', Imu,queue_size=50)

imuMsg = Imu()


#Need to find covariance values through testing. So blank right now
#Subscriber will need to check to see: if element 0 is -1, then disregard this matrix

imuMsg.orientation_covariance = [-1 , 0 , 0,
0, 0, 0,
0, 0, 0]
imuMsg.angular_velocity_covariance = [-1, 0 , 0,
0 , 0, 0,
0 , 0 , 0]
imuMsg.linear_acceleration_covariance = [-1 , 0 , 0,
0 , 0, 0,
0 , 0 , 0]


#rospy.sleep(5) # Sleep for 8 seconds to wait for the board to boot then only write command.

#loop and stream IMU (gyro and accel) data
while True:
    gyro_raw = sense.get_gyroscope_raw()
    #values from gyro_raw already are in floats, this is publishing live as well
    imuMsg.angular_velocity.x = gyro_raw["x"] #gyro
    imuMsg.angular_velocity.y = gyro_raw["y"]
    imuMsg.angular_velocity.z = gyro_raw["z"]


    #values from accel (convert Gs to m/s^2, keep as floats)
    accel_raw = sense.get_accelerometer_raw()

    imuMsg.linear_acceleration.x = (accel_raw["x"] * gtoaccel) # tripe axis accelerator meter
    imuMsg.linear_acceleration.y = (accel_raw["y"] * gtoaccel)
    imuMsg.linear_acceleration.z = (accel_raw["z"] * gtoaccel)


    

        


    #publish message with timestamp
    imuMsg.header.stamp= rospy.Time.now()
    imuMsg.header.frame_id = 'base_link'
    pub.publish(imuMsg)





 

           
      



            

