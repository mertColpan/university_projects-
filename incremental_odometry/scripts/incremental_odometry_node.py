#!/usr/bin/env python3

import rospy
import math
import random
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class IncrementalOdometry:
    def __init__(self):
        rospy.init_node('incremental_odometry_node', anonymous=True)

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.particle_pub = rospy.Publisher('/particles', PoseArray, queue_size=10)
        self.alpha = [0.001, 0.001, 0.001, 0.001]  

        self.previous_odom = None
        self.current_particles = []  
        self.num_particles = 100

    def odom_callback(self, msg):
        if self.previous_odom is None:
            self.previous_odom = msg
            return

        # Odometri farkları
        delta = self.calculate_deltas(self.previous_odom, msg)
        self.previous_odom = msg

        # Hareket modeline dayalı tahmin yap
        self.current_particles = self.sample_particles(delta)

        # Parçacıkları PoseArray olarak yayınla
        particle_array = PoseArray()
        particle_array.header.stamp = rospy.Time.now()
        particle_array.header.frame_id = "odom"  # Uygun referans çerçevesini kullanın
        particle_array.poses = self.current_particles

        self.particle_pub.publish(particle_array)

    def calculate_deltas(self, previous, current):
        # Pozisyonlar ve yönelimler
        x_prev = previous.pose.pose.position.x
        y_prev = previous.pose.pose.position.y
        theta_prev = euler_from_quaternion([
            previous.pose.pose.orientation.x,
            previous.pose.pose.orientation.y,
            previous.pose.pose.orientation.z,
            previous.pose.pose.orientation.w
        ])[2]

        x_cur = current.pose.pose.position.x
        y_cur = current.pose.pose.position.y
        theta_cur = euler_from_quaternion([
            current.pose.pose.orientation.x,
            current.pose.pose.orientation.y,
            current.pose.pose.orientation.z,
            current.pose.pose.orientation.w
        ])[2]

        # Hareket parametreleri hesaplama
        delta_rot1 = math.atan2(y_cur - y_prev, x_cur - x_prev) - theta_prev
        delta_trans = math.sqrt((x_cur - x_prev)**2 + (y_cur - y_prev)**2)
        delta_rot2 = theta_cur - theta_prev - delta_rot1

        return delta_rot1, delta_trans, delta_rot2

    def sample_particles(self, delta):
        particles = []
        delta_rot1, delta_trans, delta_rot2 = delta

        for _ in range(self.num_particles):
            # Gürültü ekleme
            delta_rot1_hat = delta_rot1 + self.sample(self.alpha[0] * delta_rot1**2 + self.alpha[1] * delta_trans**2)
            delta_trans_hat = delta_trans + self.sample(self.alpha[2] * delta_trans**2 + self.alpha[3] * (delta_rot1**2 + delta_rot2**2))
            delta_rot2_hat = delta_rot2 + self.sample(self.alpha[0] * delta_rot2**2 + self.alpha[1] * delta_trans**2)

            # Yeni konum hesaplama
            x_new = self.previous_odom.pose.pose.position.x + delta_trans_hat * math.cos(self.previous_odom.pose.pose.orientation.z + delta_rot1_hat)
            y_new = self.previous_odom.pose.pose.position.y + delta_trans_hat * math.sin(self.previous_odom.pose.pose.orientation.z + delta_rot1_hat)
            theta_new = delta_rot1_hat + delta_rot2_hat

            # Parçacık oluşturma
            particle = Pose()
            particle.position.x = x_new
            particle.position.y = y_new
            quat = quaternion_from_euler(0, 0, theta_new)
            particle.orientation = Quaternion(*quat)
            particles.append(particle)

        return particles

    def sample(self, variance):
        b = math.sqrt(variance)
        return math.sqrt(6) / 2 * (random.uniform(-b, b) + random.uniform(-b, b))

if __name__ == '__main__':
    try:
        IncrementalOdometry()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

