import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import numpy as np

# Path al bag
bag_path = "mag_calib"

# Tipo de mensaje
msg_type = get_message("sensor_msgs/msg/MagneticField")

# Abrir bag
storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
converter_options = rosbag2_py.ConverterOptions("", "")

reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

# Filtrar topic
topic = "/imu/mag"

mx_list = []
my_list = []
mz_list = []

while reader.has_next():
    topic_name, data, t = reader.read_next()
    
    if topic_name == topic:
        msg = deserialize_message(data, msg_type)
        
        mx_list.append(msg.magnetic_field.x)
        my_list.append(msg.magnetic_field.y)
        mz_list.append(msg.magnetic_field.z)

# Convertir a numpy
mx = np.array(mx_list)
my = np.array(my_list)
mz = np.array(mz_list)

# Calcular bias (hard-iron)
bias_x = (mx.max() + mx.min()) / 2.0
bias_y = (my.max() + my.min()) / 2.0
bias_z = (mz.max() + mz.min()) / 2.0

# Radios (para diagnóstico)
rx = (mx.max() - mx.min()) / 2.0
ry = (my.max() - my.min()) / 2.0
rz = (mz.max() - mz.min()) / 2.0

print("\n--- MAGNETOMETER CALIBRATION ---")
print(f"Samples: {len(mx)}")

print("\nBias (Tesla):")
print(f"mag_bias_x: {bias_x}")
print(f"mag_bias_y: {bias_y}")
print(f"mag_bias_z: {bias_z}")

print("\nRadii (for sanity check):")
print(f"rx: {rx}, ry: {ry}, rz: {rz}")

print("\nSuggested YAML:")
print(f"""
mag_bias_x: {bias_x}
mag_bias_y: {bias_y}
mag_bias_z: {bias_z}
""")
