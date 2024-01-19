import math

from matplotlib import pyplot as plt
import numpy as np

### t, avx, avy, avz, lvx, lvy, lvz, lax, lay, laz
result_path = "/home/zhonglingjun/GXT/Fusion_ROS/src/dataloader/data/write/AnalysisDataPOSE.txt"
result_data = np.zeros([10, 0])
with open(result_path, "r") as f:
    for line in f.readlines():
        data = line.split(" ")
        result_data = np.concatenate((result_data, np.array(data, dtype=float).reshape((10, 1))), axis=1)

print("loaded data!")

## Plot res
x = result_data[0, :]
# x = np.arange(result_data.shape[1])

# result_data[1:4, :] = result_data[1:4, :] * 180. / math.pi

import math
from scipy.spatial.transform import Rotation


def rotvector2ea(rotation_vector):
    # 通过旋转向量创建四元数
    quaternion = Rotation.from_rotvec(rotation_vector)
    # 通过四元数获取欧拉角
    euler_angles = quaternion.as_euler('zyx', degrees=True)  # ZYX顺序，返回角度表示
    return euler_angles


## ea
ea_list = np.zeros([3, 0])
for i in range(result_data.shape[1]):
    rt = result_data[1:4, i]
    ea = rotvector2ea(rt)
    # ea = ea * 180. / math.pi
    ea_list = np.concatenate((ea_list, np.array(ea, dtype=float).reshape((3, 1))) , axis=1)

    # print(ea)
plt.title("ea C / s")
plt.plot(x, ea_list[0, :], 'r')
plt.plot(x, ea_list[1, :], 'g')
plt.plot(x, ea_list[2, :], 'b')
plt.legend(['roll', 'pitch', 'yaw'])
plt.tight_layout()
plt.show()

# plt.subplot(311)
plt.title("angular_velocity C / s")
plt.plot(x, result_data[0 + 1, :], 'r')
plt.plot(x, result_data[1 + 1, :], 'g')
plt.plot(x, result_data[2 + 1, :], 'b')
plt.legend(['avx', 'avy', 'avz'])
plt.tight_layout()
plt.show()

# plt.subplot(312)
plt.title("linear_velocity m / s")
plt.plot(x, result_data[0 + 4, :], 'r')
plt.plot(x, result_data[1 + 4, :], 'g')
plt.plot(x, result_data[2 + 4, :], 'b')
plt.legend(['lvx', 'lvy', 'lvz'])
plt.tight_layout()
plt.show()

# plt.subplot(313)
plt.title("linear_acceleration m / s^2")
plt.plot(x, result_data[0 + 7, :], 'r')
plt.plot(x, result_data[1 + 7, :], 'g')
plt.plot(x, result_data[2 + 7, :], 'b')
plt.legend(['lax', 'lay', 'laz'])
plt.tight_layout()
plt.show()


def AllPlot():
    plt.subplot(311)
    plt.plot(x, result_data[0 + 1, :], 'r')
    plt.plot(x, result_data[1 + 1, :], 'g')
    plt.plot(x, result_data[2 + 1, :], 'b')
    plt.legend(['avx', 'avy', 'avz'])

    plt.subplot(312)
    plt.plot(x, result_data[0 + 4, :], 'r')
    plt.plot(x, result_data[1 + 4, :], 'g')
    plt.plot(x, result_data[2 + 4, :], 'b')
    plt.legend(['lvx', 'lvy', 'lvz'])

    plt.subplot(313)
    plt.plot(x, result_data[0 + 7, :], 'r')
    plt.plot(x, result_data[1 + 7, :], 'g')
    plt.plot(x, result_data[2 + 7, :], 'b')
    plt.legend(['lax', 'lay', 'laz'])
    plt.tight_layout()
    plt.show()


AllPlot()
