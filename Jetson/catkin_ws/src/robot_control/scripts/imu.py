import time
import board
import rospy
import adafruit_mpu6050
import threading

from std_msgs.msg import Empty
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler

class imuDevice():

    def __init__(self):
        
        i2c = board.I2C()
        self.imu = adafruit_mpu6050.MPU6050(i2c)

        self.pub_imu = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.pub_start_engines = rospy.Publisher('start_engines', Empty, queue_size=10)

        rospy.loginfo("Calibrando IMU, não mexer no robô! ! ! ! ! ! !")

        self.z_offset = 0
        self.sample = 100

        for _ in range(self.sample):
            self.z_offset += self.imu.gyro[2]  
            time.sleep(0.01)

        self.z_offset /= self.sample

        rospy.loginfo(f"Calibragem concluída! Erro: {self.z_offset:.4f}")

        self.yaw_rad = 0.0
        self.previous_time = time.time()

        self.running = True
        self.read_thread = threading.Thread(target=self.update)
        self.read_thread.daemon = True
        self.read_thread.start()

        self.pub_start_engines.publish(Empty())
        time.sleep(1)

    
    def calculate_yaw(self):

        current_time = time.time()
        dt = current_time - self.previous_time
        self.previous_time = current_time

                
        gyro_z_real_rad = self.imu.gyro[2] - self.z_offset
        self.yaw_rad += (gyro_z_real_rad * dt)

        imu_msg = Imu()

        q = quaternion_from_euler(0.0, 0.0, self.yaw_rad)

        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]

        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = gyro_z_real_rad

        self.pub_imu.publish(imu_msg)


    def update(self):

        rate = rospy.Rate(50)

        while not rospy.is_shutdown():

            self.calculate_yaw()
            rate.sleep()


if __name__ == "__main__":
    
    try:
        rospy.init_node('imu_node', anonymous=True)
        imu_device = imuDevice()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass