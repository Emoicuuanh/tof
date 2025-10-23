#!/usr/bin/env python3
from pymodbus.client.sync import ModbusSerialClient
from pymodbus.exceptions import ModbusException
import time, csv, rospy, struct, math, datetime, os
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import tf2_ros
import geometry_msgs.msg

# ================== CẤU HÌNH ==================
CSV_FILE = os.path.join(os.path.dirname(__file__), "modbus_log.csv")
SERIAL_PORT = "/dev/ttyACM0"
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

# ====== KHỞI TẠO FILE LOG ======
if not os.path.exists(CSV_FILE) or os.path.getsize(CSV_FILE) == 0:
    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "slave_id", "status", "message"])

# ====== HÀM GHI LOG (INFO + ERROR) ======
def log(slave_id, status, message=""):
    """Ghi log CSV cho cả trạng thái INFO và ERROR"""
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(CSV_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([timestamp, slave_id, status, message])

# ====== HÀM XỬ LÝ MODBUS ======
def process_FC3(client, slave_id, start_address, num_registers):
    try:
        response = client.read_holding_registers(start_address, num_registers, unit=slave_id)
        if response.isError():
            error_msg = f"Error reading registers: {response}"
            print(f"[Slave {slave_id}] ❌ {error_msg}")
            log(slave_id, "ERROR", error_msg)
            return None
        return response.registers
    except ModbusException as e:
        error_msg = f"Modbus exception: {e}"
        print(f"[Slave {slave_id}] ❌ {error_msg}")
        log(slave_id, "ERROR", error_msg)
        return None

# ====== HÀM HIỂN THỊ MA TRẬN VÀ GHI LOG ======
def print_matrix_8x8(registers, slave_id):
    if not registers or len(registers) < 64:
        msg = f"Not enough 64 elements (only {len(registers) if registers else 0})"
        print(f"[Slave {slave_id}] ❌ {msg}")
        log(slave_id, "ERROR", msg)
        return

    print(f"[Slave {slave_id}] 📊 Ma trận 8×8 khoảng cách (mm):")
    for i in range(8):
        row = registers[i * 8:(i + 1) * 8]
        print(" ".join(f"{val:5d}" for val in row))
    print()

    # Ghi log toàn bộ 64 giá trị khi OK
    data_str = ",".join(str(v) for v in registers)
    log(slave_id, "INFO", data_str)

# ====== HÀM TẠO POINTCLOUD ======
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

# ====== HÀM BROADCAST TF ======
def broadcast_tf(broadcaster, slave_id):
    offset = SLAVE_OFFSETS.get(slave_id, (0, 0, 0))
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link"
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
        timeout=0.2
    )

    if client.connect():
        print("✅ Connected to Modbus slaves via Serial!")

        # Ghi giá trị khởi tạo vào các thanh ghi
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
                elapsed = time.time() - t0
                print(f"[Slave {sid}] Time: {elapsed:.3f} s")
                log(sid, "INFO", f"Read cycle time: {elapsed:.3f} s")

            rate.sleep()
    else:
        print("❌ Failed to connect to Modbus slaves via Serial.")
        log(0, "ERROR", "Failed to connect to Modbus slaves via Serial.")
