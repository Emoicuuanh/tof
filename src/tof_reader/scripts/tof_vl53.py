#!/usr/bin/env python3
import rospy
import serial
import math
import json
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import time


class VL53Master:
    def __init__(self):
        rospy.init_node("vl53_master_node", anonymous=True)

        # Serial port RS485
        port = rospy.get_param("~port", "/dev/ttyACM0")
        baud = rospy.get_param("~baud", 115200)
        self.ser = serial.Serial(port, baud, timeout=0.05)

        # Cấu hình cảm biến
        self.rows = 8
        self.cols = 8
        self.hfov = math.radians(45.0)
        self.vfov = math.radians(45.0)

        # Publisher cho từng cảm biến
        self.pubs = {
            1: rospy.Publisher("/vl53_1/pointcloud", PointCloud2, queue_size=10),
            2: rospy.Publisher("/vl53_2/pointcloud", PointCloud2, queue_size=10)
        }

        # Danh sách cảm biến có ID cụ thể
        self.sensor_ids = [1, 2]
        self.rate = rospy.Rate(30)  # 10 Hz

    def request_sensor(self, sensor_id):
        """Gửi lệnh yêu cầu cảm biến có ID tương ứng"""
        cmd = f"{sensor_id}\n".encode()
        self.ser.write(cmd)
        rospy.logdebug(f"🛰️ Gửi yêu cầu đến sensor {sensor_id}")

    def read_response(self):
        """Đọc 1 dòng phản hồi JSON"""
        line = self.ser.readline().decode(errors="ignore").strip()
        return line if line else None

    def parse_and_publish(self, data):
        """Phân tích JSON và publish PointCloud"""
        try:
            parsed = json.loads(data)
            dist = parsed.get("dist", [])
            sensor_id = parsed.get("id", 0)
        except Exception as e:
            rospy.logwarn(f"Lỗi JSON: {e} | Chuỗi: {data}")
            return

        if sensor_id not in self.pubs:
            rospy.logwarn(f"Không nhận dạng được ID cảm biến: {sensor_id}")
            return

        if len(dist) != self.rows * self.cols:
            rospy.logwarn(f"Sai kích thước dữ liệu: {len(dist)} điểm (mong đợi {self.rows * self.cols})")
            return

        # Tạo pointcloud
        points = []
        for r in range(self.rows):
            for c in range(self.cols):
                d = dist[r * self.cols + c] / 1000.0
                azim = ((c - (self.cols - 1) / 2.0) / (self.cols - 1)) * self.hfov
                elev = -((r - (self.rows - 1) / 2.0) / (self.rows - 1)) * self.vfov
                x = d * math.cos(elev) * math.cos(azim)
                y = d * math.cos(elev) * math.sin(azim)
                z = d * math.sin(elev)
                points.append((x, y, z))

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = f"vl53_{sensor_id}_frame"
        cloud = pc2.create_cloud_xyz32(header, points)
        self.pubs[sensor_id].publish(cloud)

        # rospy.loginfo_throttle(0.05,f"📡 Nhận dữ liệu từ sensor {sensor_id}: dist[0..5]={dist[:6]}")

    def run(self):
        """Vòng lặp chính: gửi tuần tự từng cảm biến"""
        while not rospy.is_shutdown():
            for sensor_id in self.sensor_ids:
                # 1️⃣ Gửi lệnh yêu cầu
                self.request_sensor(sensor_id)
                # 2️⃣ Đọc dữ liệu JSON
                line = self.read_response()
                if line:
                    self.parse_and_publish(line)


if __name__ == "__main__":
    try:
        node = VL53Master()
        node.run()
    except rospy.ROSInterruptException:
        pass
