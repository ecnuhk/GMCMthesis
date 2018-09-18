from scipy import optimize, math
from numpy.linalg import norm
import numpy as np


class Answer:
    def __init__(self):
        # 高度离散化
        self.height_span = 50
        # 精度限制
        self.error = 50
        # 各个平面交点
        self.X = {}

        # 虚假轨迹点
        xs = [60600, 61197, 61790, 62377, 62955, 63523, 64078, 64618, 65141, 65646, 66131, 66594, 67026, 67426, 67796,
              68134, 68442, 68719, 68966, 69184]
        ys = [69982, 69928, 69838, 69713, 69553, 69359, 69131, 68870, 68577, 68253, 67900, 67518, 67116, 66697, 66263,
              65817, 65361, 64897, 64429, 63957]
        zs = [7995, 7980, 7955, 7920, 7875, 7820, 7755, 7680, 7595, 7500, 7395, 7280, 7155, 7020, 6875, 6720, 6555,
              6380,
              6195, 6000]
        # 雷达
        x = [80000, 30000, 55000, 105000, 130000]
        y = [0, 60000, 110000, 110000, 60000]
        z = [0, 0, 0, 0, 0]

        # P为虚假轨迹点
        self.P = []
        for i in range(len(xs)):
            self.P.append({
                'x': xs[i],
                'y': ys[i],
                'z': zs[i]
            })
        # print(self.P)

        # R为雷达点
        self.R = []
        for i in range(len(x)):
            self.R.append({
                'x': x[i],
                'y': y[i],
                'z': z[i]
            })
            # print(self.R)

    # 计算所有等高面交点
    def get_points_by_all_height(self):
        for height in range(2000, 2500 + 1, self.height_span):
            self.get_points_by_height(height)

    # 计算某个等高面上交点
    def get_points_by_height(self, height):
        X = self.X

        X[height] = []
        for i in range(5):
            X_Ri = []
            for j in range(20):
                X_Ri.append(self.get_point_by_height(i, j, height))
            X[height].append(X_Ri)
        self.X = X
        return X

    # 计算等高面上1个交点
    def get_point_by_height(self, Ri, Pi, height):
        z = height
        x1 = self.R[Ri]['x']
        y1 = self.R[Ri]['y']
        z1 = self.R[Ri]['z']

        x2 = self.P[Pi]['x']
        y2 = self.P[Pi]['y']
        z2 = self.P[Pi]['z']

        x = (x2 - x1) * (z - z1) / (z2 - z1) + x1
        y = (y2 - y1) * (z - z1) / (z2 - z1) + y1
        # print(x, y, z)
        return {'x': round(x, 2), 'y': round(y, 2), 'z': round(z, 2)}

    # 直飞点计算
    def get_legal_flight_by_height(self, height):
        X = self.X
        lines = []
        Hk = height
        for Ri in range(5):
            for Pj in range(20):  # 为每个点找到所有可能直线
                # 起始点
                x0 = X[Hk][Ri][Pj]['x']
                y0 = X[Hk][Ri][Pj]['y']

                # 一条飞行方案
                rst = {
                    'Ri': Ri,
                    'Pj': Pj,
                    'Hk': Hk,
                    'X': [{'x': x0, 'y': y0, 't': Pj}],  # 方案路过的所有点
                    'para': None  # 方案的直线参数
                }
                for dirc in range(Pj + 1, 20):  # 尝试从Pj出发的所有方向
                    # 确定方向
                    x1 = X[Hk][Ri][dirc]['x']
                    y1 = X[Hk][Ri][dirc]['y']

                    # 判断能否飞到
                    if not self.is_dirc_ok(x0, y0, x1, y1, (dirc - Pj) * 10):
                        continue

                    # 加入方向点
                    line_x = [x0, x1]
                    line_y = [y0, y1]
                    line_t = [Pj, dirc]  # 时间序列点

                    # 直线拟合与绘制
                    A, B = optimize.curve_fit(lambda x, A, B: A * x + B, line_x, line_y)[0]

                    # 判断后续是否有共线点
                    for each in range(dirc + 1, 20):
                        x_ = X[Hk][Ri][each]['x']
                        y_ = X[Hk][Ri][each]['y']
                        if self.is_collinear(A, B, x_, y_):
                            # 和前面共线，加入尝试集合中
                            line_x.append(x_)
                            line_y.append(y_)
                            line_t.append(each)

                            # 判断是否满足运动学要求
                            if not self.is_line_ok(line_x, line_y, line_t):
                                line_x = line_x[:-1]
                                line_y = line_y[:-1]
                                line_t = line_t[:-1]

                    # 生成方案
                    rst['para'] = {'A': A, 'B': B}
                    for i in range(len(line_t)):
                        if i == 0:
                            continue
                        rst['X'].append({'x': line_x[i], 'y': line_y[i], 't': line_t[i]})
                # 加入方案
                lines.append(rst)
        return lines

    # 计算平面2点距
    def get_distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    # 根据2点和时间间隔判断某方位可行性
    def is_dirc_ok(self, x1, y1, x2, y2, delta_t):
        v = self.get_distance(x1, y1, x2, y2) / delta_t
        return 120 / 3.6 <= v and v <= 180 / 3.6

    # 由前2点判断多点的运动学可行性
    def is_line_ok(self, xs, ys, ts):
        # 直线拟合与绘制
        # A, B = optimize.curve_fit(lambda x, A, B: A * x + B, xs, ys)[0]

        # 计算运动速度
        v_x = (xs[1] - xs[0]) / ((ts[1] - ts[0]) * 10)
        v_y = (ys[1] - ys[0]) / ((ts[1] - ts[0]) * 10)

        # 由速度判断实际飞行点是否到达
        for i in range(2, len(ts)):
            x_ = xs[0] + v_x * ((ts[i] - ts[0]) * 10)
            y_ = ys[0] + v_y * ((ts[i] - ts[0]) * 10)

            # 判断点点误差
            if self.get_distance(xs[i], ys[i], x_, y_) > self.error:
                return False

        return True

    def is_collinear(self, A, B, x, y):
        if isinstance(x, int):
            return math.fabs(A * x + B - y) <= self.error

        if isinstance(x, list):
            for i in range(len(x)):
                if math.fabs(A * x[i] + B - y[i]) > self.error:
                    return False
            return True


# 输出所有航线
def paint_lines():
    import matplotlib.pyplot as plt
    answer = Answer()
    answer.get_points_by_all_height()

    fig = plt.figure()

    for height in range(2000, 2501, 100):

        ax = fig.add_subplot(2, 3, (height - 2000) / 100 + 1)
        ax.set_xlabel('X Label', color='r')
        ax.set_ylabel('Y Label', color='g')
        for Ri in range(len(answer.X[height])):
            each = answer.X[height][Ri]
            for Pj in range(len(each)):
                each2 = each[Pj]
                s = ax.scatter(each2['x'], each2['y'], c='r', marker='o', label=height)
                # ax.text(each2['x'], each2['y'], "(%d,%d)" % (each2['x'], each2['y']))
                ax.text(each2['x'], each2['y'], "(%d,%d)" % (Ri, Pj))

        ax.legend([s], ['z: ' + str(height)])
        lines = answer.get_legal_flight_by_height(height)

        line_count = 1
        for each in lines:
            # print(each['Hk'], each['Ri'], each['Pj'], each['para'], sep='\t', end='\t')
            if each['para'] is not None:
                # A = each['para']['A']
                # B = each['para']['B']
                # x = [each['X'][0]['x'], each['X'][-1]['x']]
                # x = np.array(x)
                # y = A * x + B
                # ax.plot(x, y)
                x = []
                y = []
                for i in range(len(each['X'])):
                    x.append(each['X'][i]['x'])
                    y.append(each['X'][i]['y'])
                ax.plot(x, y, linewidth=line_count, linestyle='dashed', alpha=0.7)
                line_count += 0.2
                # for Xs in each['X']:
                #     print((Xs['t'], Xs['x'], Xs['y']), end='\t')
                # print()

    plt.show()


# 输出全交点坐标
def get_all_points():
    answer = Answer()
    answer.get_points_by_all_height()
    for height in answer.X.keys():
        # print(height)
        for Ri in range(len(answer.X[height])):
            R = answer.X[height][Ri]
            for Pj in range(len(R)):
                P = R[Pj]
                print(Ri + 1, Pj + 1, P['x'], P['y'], P['z'])
                # print()


if __name__ == '__main__':
    paint_lines()
