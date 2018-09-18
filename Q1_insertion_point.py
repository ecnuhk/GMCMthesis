# 全局连线图

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from Q1 import Answer

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# 虚假轨迹点
xs = [60600, 61197, 61790, 62377, 62955, 63523, 64078, 64618, 65141, 65646, 66131, 66594, 67026, 67426, 67796,
      68134, 68442, 68719, 68966, 69184]
ys = [69982, 69928, 69838, 69713, 69553, 69359, 69131, 68870, 68577, 68253, 67900, 67518, 67116, 66697, 66263,
      65817, 65361, 64897, 64429, 63957]
zs = [7995, 7980, 7955, 7920, 7875, 7820, 7755, 7680, 7595, 7500, 7395, 7280, 7155, 7020, 6875, 6720, 6555, 6380,
      6195, 6000]
# 雷达
x = [80000, 30000, 55000, 105000, 130000]
y = [0, 60000, 110000, 110000, 60000]
z = [0, 0, 0, 0, 0]


def main():
    background()
    radar()
    lines()
    height()
    insertion_point()

    ax.set_xlabel('X Label', color='r')
    ax.set_ylabel('Y Label', color='g')
    ax.set_zlabel('Z Label', color='b')
    plt.show()


# 虚假轨迹绘图
def background():
    ax.scatter(xs, ys, zs, c='r', marker='o')


# 雷达绘图
def radar():
    ax.scatter(x, y, z, c='b', marker='o')


# 雷达-虚假点连线
def lines():
    for radar_id in range(5):
        color = ['violet'] * 5  # ['violet','lightsteelblue','teal','olive','red']
        for i in range(len(xs)):
            pass
            ax.plot([xs[i], x[radar_id]], [ys[i], y[radar_id]], [zs[i], z[radar_id]], c=color[radar_id], alpha=0.5)


# z=2k~2.5k
def height():
    X = np.arange(0, 120000, 5000)
    Y = np.arange(0, 120000, 5000)
    X, Y = np.meshgrid(X, Y)
    Z = X + 2000 - X
    surf = ax.plot_surface(X, Y, Z, rstride=1, cmap=cm.ocean, cstride=1, linewidth=0, antialiased=False, alpha=0.5)
    Z = X + 2500 - X
    #surf = ax.plot_surface(X, Y, Z, rstride=1, cmap=cm.terrain, cstride=1, linewidth=0, antialiased=False, alpha=0.5)

# 绘制交点
def insertion_point():
    ans = Answer()
    X = ans.get_points_by_height(2000)
    for Ri in range(5):
        for Pj in range(20):
            x = X[Ri][Pj]['x']
            y = X[Ri][Pj]['y']
            z = X[Ri][Pj]['z']
            ax.plot([x], [y], [z], c='b', marker='o', markersize=1)


if __name__ == '__main__':
    main()
