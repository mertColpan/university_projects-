#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class WallFollower:
    def __init__(self, d=0.95, r=0.6):
        self.d = d  # Duvara mesafesi
        self.r = r  # Tolerans aralığı

        # cmd_vel icin Publisher, odom ve lazer sensoru icin Subscriber tanimalamasi
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.combined_callback)
        rospy.Subscriber('/scan', LaserScan, self.combined_callback)

        # Hareket komutları için Twist nesnesi
        self.move_cmd = Twist()

        # Son odometry ve lazer verilerini saklamak icin
        self.latest_odom = None
        self.latest_scan = None
        
    def combined_callback(self, data):
        # Gelen mesajin turune gore ilgili degiskeni guncelle 
        if isinstance(data, LaserScan):
            self.latest_scan = data
        elif isinstance(data, Odometry):
            self.latest_odom = data

        # Hem odom hem de lazer verisi geldiyse isleme devam et
        if self.latest_odom and self.latest_scan:
            self.process_data()

    def process_data(self):
        # Odometry'den pozisyon bilgisi al
        position = self.latest_odom.pose.pose.position
        x, y = position.x, position.y
        #rospy.loginfo(f"Current Position -> x: {x:.2f}, y: {y:.2f}")

        # (0, 0) koordinati merkezli 2m x 2m buyuklugundeki kareye ulasilmissa robotu durdur
        if -1.0 <= x <= 1.0 and -1.0 <= y <= 1.0:
            rospy.loginfo("Goal reached! Stopping the robot.")
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(self.move_cmd)
            rospy.signal_shutdown("Goal reached")
            return

        # Lazerden mesafe bilgilerini belirli derece araliklarinda minimumuna bakarak daha tutarli verilerini al
        front_distance = min(self.latest_scan.ranges[0:15])
        right_distance = min(self.latest_scan.ranges[315:359])
        left_distance = min(self.latest_scan.ranges[15:45])

        rospy.loginfo(f"Ön: {front_distance:.2f} m, Sol: {left_distance:.2f} m, Sağ: {right_distance:.2f} m")

        # Hareket mantigini uygulama
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.0

        if front_distance < 0.80:
            self.move_cmd.angular.z = 0.3  # RObotun onunde engel varsa don
        else:
            if right_distance < (self.d + self.r):
                if right_distance < (self.d - self.r):
                    self.move_cmd.linear.x = 0.1
                    self.move_cmd.angular.z = 0.2  # Duvara yakinsa uzaklas
                else:
                    self.move_cmd.linear.x = 0.3  # Ideal mesafede ise daha hizli git 
            else:
                self.move_cmd.linear.x = 0.1
                self.move_cmd.angular.z = -0.2  # Duvara uzaksa yakinlas

        # Hareket komutunu yayinla
        self.cmd_vel_pub.publish(self.move_cmd)

def main():
    rospy.init_node('wall_follower')
    node = WallFollower(d=0.5, r=0.1)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
