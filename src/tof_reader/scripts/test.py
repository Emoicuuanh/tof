#!/usr/bin/env python3
from pymodbus.client.sync import ModbusSerialClient
from pymodbus.exceptions import ModbusException
import time
import csv
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import tf2_ros
import geometry_msgs.msg
import math

# ================== CẤU HÌNH ==================
CSV_FILE = "modbus_log.csv"
SERIAL_PORT = "/dev/ttyS1"
BAUDRATE = 57600
WRITE_ADDRESS = 130
WRITE_LENGTH = 64
NUM_REGISTERS = 65

SLAVE_IDS = [1, 2]

# Offset vị trí của từng slave so với base_link
SLAVE_OFFSETS = {
    1: (0.0, 1.0, 0.0),
    2: (0.0, -1.0, 0.0)
}
# ==============================================


def process_FC3(client, slave_id, start_address, num_registers):
    try:
        response = client.read_holding_registers(start_address, num_registers, unit=slave_id)
        if response.isError():
            print(f"[Slave {slave_id}] Error reading registers: {response}")
            return None
        return response.registers
    except ModbusException as e:
        print(f"[Slave {slave_id}] Modbus error: {e}")
        return None


def print_matrix_8x8(registers, slave_id):
    if not registers or len(registers) < 64:
        print(f"[Slave {slave_id}] ❌ Dữ liệu không đủ 64 phần tử để hiển thị 8x8.")
        return

    print(f"[Slave {slave_id}] 📊 Ma trận 8×8 khoảng cách (mm):")
    for i in range(8):
        row = registers[i * 8:(i + 1) * 8]
        print(" ".join(f"{val:5d}" for val in row))
    print()


def create_pointcloud(slave_id, distances):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = f"slave_{slave_id}"

    points = []
    rows, cols = 8, 8
    hfov = math.radians(45.0)
    vfov = math.radians(45.0)

    for i in range(rows):
        for j in range(cols):
            idx = i * cols + j
            if idx < len(distances):
                d = distances[idx] / 1000.0  # mm → m
                azim = ((j - (cols - 1) / 2.0) / (cols - 1)) * hfov
                elev = -((i - (rows - 1) / 2.0) / (rows - 1)) * vfov
                x = d * math.cos(elev) * math.cos(azim)
                y = d * math.cos(elev) * math.sin(azim)
                z = d * math.sin(elev)
                points.append([x, y, z])

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]

    data = b''.join([struct.pack('fff', *p) for p in points])
    return PointCloud2(
        header=header,
        height=1,
        width=len(points),
        is_dense=True,
        is_bigendian=False,
        fields=fields,
        point_step=12,
        row_step=12 * len(points),
        data=data
    )



def broadcast_tf(broadcaster, slave_id):
    offset = SLAVE_OFFSETS.get(slave_id, (0, 0, 0))
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link"       # ✅ Frame gốc là base_link
    t.child_frame_id = f"slave_{slave_id}"
    t.transform.translation.x = offset[0]
    t.transform.translation.y = offset[1]
    t.transform.translation.z = offset[2]
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0
    broadcaster.sendTransform(t)


# ================== CHƯƠNG TRÌNH CHÍNH ==================
if __name__ == "__main__":
    rospy.init_node("tof_modbus_dual_publisher")

    # ✅ Hai topic riêng cho 2 cảm biến
    pub1 = rospy.Publisher("/tof_points_1", PointCloud2, queue_size=2)
    pub2 = rospy.Publisher("/tof_points_2", PointCloud2, queue_size=2)
    publishers = {1: pub1, 2: pub2}

    br = tf2_ros.TransformBroadcaster()

    client = ModbusSerialClient(
        method='rtu',
        port=SERIAL_PORT,
        baudrate=BAUDRATE,
        bytesize=8,
        parity='N',
        stopbits=1,
        timeout=0.1
    )

    if client.connect():
        print("✅ Connected to Modbus slaves via Serial!")

        init_vals = [60] * WRITE_LENGTH
        for sid in SLAVE_IDS:
            client.write_registers(WRITE_ADDRESS, init_vals, unit=sid)

        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            for sid in SLAVE_IDS:
                print(f"\n===== Đọc dữ liệu từ Slave {sid} =====")
                t0 = time.time()

                registers = process_FC3(client, sid, 65, 64)
                if registers:
                    print_matrix_8x8(registers, sid)
                    cloud = create_pointcloud(sid, registers)
                    publishers[sid].publish(cloud)

                broadcast_tf(br, sid)
                print(f"[Slave {sid}] Time: {time.time() - t0:.3f} s")

            rate.sleep()
    else:
        print("❌ Failed to connect to Modbus slaves via Serial.")