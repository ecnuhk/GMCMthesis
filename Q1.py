import copy

from scipy import optimize, math
from numpy.linalg import norm
import numpy as np
import matplotlib.pyplot as plt


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

                for dirc in range(Pj + 1, 20):  # 尝试从Pj出发的所有方向
                    line_x, line_y, line_t = [x0], [y0], [Pj]

                    # 确定方向
                    x1 = X[Hk][Ri][dirc]['x']
                    y1 = X[Hk][Ri][dirc]['y']

                    # 判断能否飞到
                    if not self.is_dirc_ok(x0, y0, x1, y1, (dirc - Pj) * 10):
                        continue

                    # 一条飞行方案
                    rst = {
                        'Ri': Ri,
                        'Pj': Pj,
                        'Hk': Hk,
                        'X': [],  # 方案路过的所有点
                        'X_idx': [(Ri, Pj, Hk), (Ri, dirc, Hk)],
                        'para': None  # 方案的直线参数
                    }

                    # 加入方向点
                    line_x.append(x1)
                    line_y.append(y1)
                    line_t.append(dirc)  # 时间序列点

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

    # 输出所有航线，并绘制
    def paint_lines(self):
        self.get_points_by_all_height()

        fig = plt.figure()

        for height in range(2000, 2501, 100):

            ax = fig.add_subplot(2, 3, (height - 2000) / 100 + 1)
            ax.set_xlabel('X Label', color='r')
            ax.set_ylabel('Y Label', color='g')
            for Ri in range(len(self.X[height])):
                each = self.X[height][Ri]
                for Pj in range(len(each)):
                    each2 = each[Pj]
                    s = ax.scatter(each2['x'], each2['y'], c='r', marker='o', label=height)
                    # ax.text(each2['x'], each2['y'], "(%d,%d)" % (each2['x'], each2['y']))
                    ax.text(each2['x'], each2['y'], "(%d,%d)" % (Ri, Pj))

            ax.legend([s], ['z: ' + str(height)])
            lines = self.get_legal_flight_by_height(height)

            for each in lines:
                print(each['Hk'], each['Ri'], each['Pj'], each['para'], sep='\t', end='\t')
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
                    ax.plot(x, y, linewidth=1, linestyle='dashed', alpha=0.5)

                    for Xs in each['X']:
                        print((Xs['t'], Xs['x'], Xs['y']), end='\t')
                    print()

        plt.show()

    # 输出所有航线
    def get_all_plans_and_paint(self):
        self.get_points_by_all_height()

        plans = []
        for height in range(2000, 2501, 100):

            lines = self.get_legal_flight_by_height(height)

            for each in lines:

                # print(each['Hk'], each['Ri'], each['Pj'], each['para'], sep='\t', end='\t')
                if each['para'] is not None:
                    print(each['X_idx'])
                    plans.append(each['X_idx'])

        import matplotlib.pyplot as plt
        fig = plt.figure()
        for height in range(2000, 2501, 100):

            ax = fig.add_subplot(2, 3, (height - 2000) / 100 + 1)
            ax.set_xlabel('X Label', color='r')
            ax.set_ylabel('Y Label', color='g')
            for Ri in range(len(self.X[height])):
                each = self.X[height][Ri]
                for Pj in range(len(each)):
                    each2 = each[Pj]
                    s = ax.scatter(each2['x'], each2['y'], c='r', marker='o', label=height)
                    # ax.text(each2['x'], each2['y'], "(%d,%d)" % (each2['x'], each2['y']))
                    ax.text(each2['x'], each2['y'], "(%d,%d)" % (Ri, Pj))

            ax.legend([s], ['z: ' + str(height)])

            for each in plans:
                if each[0][2] == height:
                    X1 = self.X[height][each[0][0]][each[0][1]]
                    X2 = self.X[height][each[1][0]][each[1][1]]
                    x = [X1['x'], X2['x']]
                    y = [X1['y'], X2['y']]
                    ax.plot(x, y)

        plt.show()

    # 输出全交点坐标
    def get_all_points(self):
        self.get_points_by_all_height()
        for height in self.X.keys():
            # print(height)
            for Ri in range(len(self.X[height])):
                R = self.X[height][Ri]
                for Pj in range(len(R)):
                    P = R[Pj]
                    print(Ri + 1, Pj + 1, P['x'], P['y'], P['z'])
                    # print()

    def get_plans(self):
        self.get_points_by_all_height()
        plans = []
        for height in range(2000, 2501, 100):
            lines = self.get_legal_flight_by_height(height)
            for each in lines:
                # print(each['Hk'], each['Ri'], each['Pj'], each['para'], sep='\t', end='\t')
                if each['para'] is not None:
                    # print(each['X_idx'])
                    plans.append(each['X_idx'])

        return plans

    def result(self):
        plans = self.get_plans()
        self.plan_by_Pj = [[], [], [], [], [], [], [], [], [], [], [], [], [], [], [], [], [], [], [], []]
        for each in plans:
            start, end = each[0], each[1]
            row = {
                'start': start,
                'end': end,
                'choose': False,
                'from': None
            }
            self.plan_by_Pj[start[1]].append(row)
            self.plan_by_Pj[end[1]].append(row)

        # for each in self.plan_by_Pj:
        #     print(each)

        # 寻解
        self.stack = []
        self.choice = [len(self.plan_by_Pj[i]) for i in range(20)]  # 每个虚假点的可选航迹数目
        self.satisfy = [0 for i in range(20)]  # 已选航迹数目
        self.answer = {
            'number': 999999,
            'answer': [None for i in range(999)]
        }
        need_pop = False
        while not self.is_satisfy():
            if need_pop:
                row = self.get_next_line()
                need_pop = False
            else:
                min_Pj = self.get_min_choice_Pj()  # 找到可选航迹数最少的虚假点
                row = self.get_line_from_Pj(min_Pj)  # 取得航迹
            if row is None:
                # 记录此状态需要无人机数量
                number = self.get_un_satisfy_num() + len(self.stack)

                # 优先选数量少的
                if number < self.answer['number']:
                    self.answer['number'] = number
                    self.answer['answer'] = self.stack[:]

                    # 打印一下
                    print('需要%d架无人机，目前栈深%d' % (number, len(self.stack)), end='\t')
                    self.print_stack()
                else:
                    # 回退
                    need_pop = True
            else:
                self.stack.append(row)
        print(self.stack)

    def is_satisfy(self):
        for each in self.satisfy:
            if each < 3:
                return False
        return True

    def get_min_choice_Pj(self):
        min_Pj = None

        # 找到第一个正数Pj
        for i in range(20):
            if self.choice[i] > 0:
                min_Pj = i

        # 没有可选项了
        if min_Pj is None:
            return None

        for i in range(20):
            if self.choice[i] < self.choice[min_Pj]:
                min_Pj = i

        return min_Pj

    def get_line_from_Pj(self, Pj):
        if Pj is None:
            return None

        for row in self.plan_by_Pj[Pj]:
            # 是否已选过
            if row['choose']:
                continue

            # 检测是否与之前点冲突
            conflict = False
            start, end = row['start'], row['end']
            for each in self.stack:
                if each == start or each == end:
                    conflict = True
            if conflict:
                continue

            # 不分配航迹到已经有无人机的点上
            if row['start'][1] == Pj and self.satisfy[row['end'][1]]>=3:  # 判断end点是否已满足
                continue
            if row['end'][1] == Pj and self.satisfy[row['start'][1]]>=3:  # 判断start点是否已满足
                continue


            row['choose'] = True
            row['from'] = Pj
            # 更新选择数
            start_Pj = row['start'][1]
            end_Pj = row['end'][1]
            # 更新选择数
            self.choice[start_Pj] -= 1
            self.choice[end_Pj] -= 1
            # 更新已满足虚假点数
            self.satisfy[start_Pj] += 1
            self.satisfy[end_Pj] += 1
            return row

        return None

    def get_un_satisfy_num(self):
        count = 0
        for each in self.satisfy:
            if each < 3:
                count += 3 - each
        return count

    def get_next_line(self):
        # 找到来源
        pre = self.stack.pop()
        start_Pj = pre['start'][1]
        end_Pj = pre['end'][1]

        # 清除访问
        pre['choose'] = False
        self.choice[start_Pj] += 1
        self.choice[end_Pj] += 1
        self.satisfy[start_Pj] -= 1
        self.satisfy[end_Pj] -= 1

        Pj = pre['from']  # 上一步的欲选择虚假点
        pre['from'] = None

        find_pre = False
        for row in self.plan_by_Pj[Pj]:
            if not find_pre:
                if row == pre:
                    find_pre = True
            else:
                row['choose'] = True
                row['from'] = Pj
                # 更新选择数
                start_Pj = row['start'][1]
                end_Pj = row['end'][1]
                # 更新选择数
                self.choice[start_Pj] -= 1
                self.choice[end_Pj] -= 1
                # 更新已满足虚假点数
                self.satisfy[start_Pj] += 1
                self.satisfy[end_Pj] += 1
                return row

        return None

    def print_stack(self):
        for each in self.stack:
            start = each['start']
            end = each['end']
            print('(%d,%d,%d)->(%d,%d,%d)' % (start[0], start[1], start[2], end[0], end[1], end[2]), end='\t')
        print()


if __name__ == '__main__':
    ans = Answer()
    ans.result()
