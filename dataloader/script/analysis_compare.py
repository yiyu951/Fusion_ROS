import math

from matplotlib import pyplot as plt
import numpy as np

### t, avx, avy, avz, lvx, lvy, lvz, lax, lay, laz
result_path_pose = "/home/zhonglingjun/GXT/Fusion_ROS/src/dataloader/data/write/AnalysisDataPOSE.txt"
result_data_pose = np.zeros([10, 0])
with open(result_path_pose, "r") as f:
    for line in f.readlines():
        data = line.split(" ")
        result_data_pose = np.concatenate((result_data_pose, np.array(data, dtype=float).reshape((10, 1))), axis=1)

result_path_imu = "/home/zhonglingjun/GXT/Fusion_ROS/src/dataloader/data/write/AnalysisDataIMU.txt"
result_data_imu = np.zeros([10, 0])
with open(result_path_imu, "r") as f:
    for line in f.readlines():
        data = line.split(" ")
        result_data_imu = np.concatenate((result_data_imu, np.array(data, dtype=float).reshape((10, 1))), axis=1)


print("loaded data!")

## Plot res
x_pose = result_data_pose[0, :]
x_imu = result_data_imu[0, :]


import math
from scipy.spatial.transform import Rotation

def rotvector2ea(rotation_vector):
    # 通过旋转向量创建四元数
    quaternion = Rotation.from_rotvec(rotation_vector)
    # 通过四元数获取欧拉角

    euler_angles = quaternion.as_euler('zyx', degrees=True)  # ZYX顺序，返回角度表示
    return euler_angles


## ea
ea_list_pose = np.zeros([3, 0])
for i in range(result_data_pose.shape[1]):
    rt = result_data_pose[1:4, i]
    ea = rotvector2ea(rt)
    ea_list_pose = np.concatenate((ea_list_pose, np.array(ea, dtype=float).reshape((3, 1))) , axis=1)

ea_list_imu = np.zeros([3, 0])
for i in range(result_data_imu.shape[1]):
    rt = result_data_imu[1:4, i]
    ea = rotvector2ea(rt)
    ea_list_imu = np.concatenate((ea_list_imu, np.array(ea, dtype=float).reshape((3, 1))) , axis=1)



plt.subplot(311)
plt.title("ea roll C/s")
plt.plot(x_pose, ea_list_pose[0, :], 'r')
plt.plot(x_imu, ea_list_imu[0, :], 'g')
plt.legend(['pose', 'imu'])

plt.subplot(312)
plt.title("ea pitch C/s")
plt.plot(x_pose, ea_list_pose[1, :], 'r')
plt.plot(x_imu, ea_list_imu[1, :], 'g')
plt.legend(['pose', 'imu'])

plt.subplot(313)
plt.title("ea yaw C/s")
plt.plot(x_pose, ea_list_pose[2, :], 'r')
plt.plot(x_imu, ea_list_imu[2, :], 'g')
plt.legend(['pose', 'imu'])

plt.tight_layout()
plt.show()

## angular_velocity
plt.subplot(311)
plt.title("angular_velocity-X rad/s")
plt.plot(x_pose, result_data_pose[1, :], 'r')
plt.plot(x_imu, result_data_imu[1, :], 'g')
plt.legend(['pose', 'imu'])

plt.subplot(312)
plt.title("angular_velocity-Y rad/s")
plt.plot(x_pose, result_data_pose[2, :], 'r')
plt.plot(x_imu, result_data_imu[2, :], 'g')
plt.legend(['pose', 'imu'])

plt.subplot(313)
plt.title("angular_velocity-Z rad/s")
plt.plot(x_pose, result_data_pose[3, :], 'r')
plt.plot(x_imu, result_data_imu[3, :], 'g')
plt.legend(['pose', 'imu'])

plt.tight_layout()
plt.show()



## linear_velocity
plt.subplot(311)
plt.title("linear_velocity X m / s")
plt.plot(x_pose, result_data_pose[4, :], 'r')
plt.plot(x_imu, result_data_imu[4, :], 'g')
plt.legend(['pose', 'imu'])

plt.subplot(312)
plt.title("linear_velocity Y m / s")
plt.plot(x_pose, result_data_pose[5, :], 'r')
plt.plot(x_imu, result_data_imu[5, :], 'g')
plt.legend(['pose', 'imu'])

plt.subplot(313)
plt.title("linear_velocity Z m / s")
plt.plot(x_pose, result_data_pose[6, :], 'r')
plt.plot(x_imu, result_data_imu[6, :], 'g')
plt.legend(['pose', 'imu'])

plt.tight_layout()
plt.show()


## linear_acceleration
plt.subplot(311)
plt.title("linear_acceleration m / s^2")
plt.plot(x_pose, result_data_pose[7, :], 'r')
plt.plot(x_imu, result_data_imu[7, :], 'g')
plt.legend(['pose', 'imu'])

plt.subplot(312)
plt.title("linear_acceleration m / s^2")
plt.plot(x_pose, result_data_pose[8, :], 'r')
plt.plot(x_imu, result_data_imu[8, :], 'g')
plt.legend(['pose', 'imu'])

plt.subplot(313)
plt.title("linear_acceleration m / s^2")
plt.plot(x_pose, result_data_pose[9, :], 'r')
plt.plot(x_imu, result_data_imu[9, :], 'g')
plt.legend(['pose', 'imu'])

plt.tight_layout()
plt.show()



