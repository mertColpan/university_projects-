#!/usr/bin/env python3

import rospy
import math
import numpy as np
import cv2
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import tf

class Mapper:
    def __init__(self):
        # Node 
        rospy.init_node('my_mapper', anonymous=True)

        # TF dinleyici (transformations)
        self.tf_listener = tf.TransformListener()

        # Lidar verilerini ve odom bilgisini dinleyen subscriberlar
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # OpenCV harita cercevesini olustur
        self.map_size = 800  # Harita penceresinin boyutu 
        self.map_resolution = 0.05  #1 pixel = 5 cm
        self.map = np.zeros((self.map_size, self.map_size, 3), dtype=np.uint8)

    #cerceve islemlerinde odom kullanilmiyor 
    def odom_callback(self, msg):
        pass

    def scan_callback(self, scan):
        # i = index, range = ranges[i] degeri
        for i, range in enumerate(scan.ranges):
            # Sonsuz ve gecersiz degerlerin atlanmasi
            if math.isinf(range) or math.isnan(range):
                continue 

            # Lidar acisi hesapla
            angle = scan.angle_min + i * scan.angle_increment

            # base_scan cercevesinin (x, y) koordinatlarini hesapla
            x = range * math.cos(angle)
            y = range * math.sin(angle)

            # base_scan -> base_footprint donusumu
            scan_point = PointStamped()
            scan_point.header.frame_id = "base_scan"
            scan_point.point.x = x
            scan_point.point.y = y
            scan_point.point.z = 0.0

            try:
                # base_footprint cercevesine gore donustur
                footprint_point = self.tf_listener.transformPoint("base_footprint", scan_point)
                # base_footprint cercevesini odom cercevesine donusturme
                odom_point = self.tf_listener.transformPoint("odom", footprint_point)
            # tf expection kontrolu
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(f"Transform hatası: {e}")
                continue

            # Odom cercevesindeki (x, y) koordinatlarini harita penceresine cevir
            map_x = int(odom_point.point.x / self.map_resolution + self.map_size / 2)
            map_y = int(odom_point.point.y / self.map_resolution + self.map_size / 2)

            if 0 <= map_x < self.map_size and 0 <= map_y < self.map_size:
                # Haritaya cizim
                cv2.circle(self.map, (map_x, map_y), 1, (255, 255, 255), -1)

        # OpenCV penceresinde haritayi goster
        cv2.imshow("TurtleBot3 Mapping", self.map)
        cv2.waitKey(1)  # Pencereyi guncelle

if __name__ == "__main__":
    try:
        mapper = Mapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
