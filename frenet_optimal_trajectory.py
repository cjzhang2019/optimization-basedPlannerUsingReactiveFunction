# coding=utf-8
import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import cubic_spline
import vehicle
import collision_check
import scipy.spatial
import get_boundaries_centerline
import time

# Parameter
MAX_SPEED = 50.0 / 3.6  # 最大速度 [m/s]
MAX_ACCEL = 2.0  # 最大加速度[m/ss]
MAX_CURVATURE = 1.0  # 最大曲率 [1/m]
TOP_ROAD_WIDTH = 5  # 最大道路宽度 [m]
BOTTOM_ROAD_WIDTH = -7.0  # 最大道路宽度 [m]
D_ROAD_W = 1.0  # 道路宽度采样间隔 [m]
DT = 0.2  # Delta T [s]
MAXT = 5.0  # 最大预测时间 [s]
MINT = 4.0  # 最小预测时间 [s]
TARGET_SPEED = 10.0 / 3.6  # 目标速度（即纵向的速度保持） [m/s]
D_T_S = 5.0 / 3.6  # 目标速度采样间隔 [m/s]
N_S_SAMPLE = 1  # sampling number of target speed
VEHICLE_RADIUS = 1.5  # robot radius [m]

# 损失函数权重
KJ = 0.1
KT = 0
KD = 1.0
KLAT = 1.0
KLON = 1.0


class quintic_polynomial:
    def __init__(self, xs, vxs, axs, xe, vxe, axe, T):
        # 这里输入为初始的x,v,a以及目标的x,v,a和时间t1-t0,Takahashi的文章——Local path planning and motion control for AGV in positioning中已经证明，任何Jerk最优化问题中的解都可以使用一个5次多项式来表示
        # 计算五次多项式系数
        
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.xe = xe
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[T ** 3, T ** 4, T ** 5],
                      [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                      [6 * T, 12 * T ** 2, 20 * T ** 3]])
        b = np.array([xe - self.a0 - self.a1 * T - self.a2 * T ** 2,
                      vxe - self.a1 - 2 * self.a2 * T,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]
        #这里输出为5次多项式的6个系数
    #以下通过这个5次函数反推轨迹点各信息
    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4 + self.a5 * t ** 5

        return xt

    def calc_first_derivative(self, t):
        dxt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3 + 5 * self.a5 * t ** 4

        return dxt

    def calc_second_derivative(self, t):
        ddxt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2 + 20 * self.a5 * t ** 3

        return ddxt

    def calc_third_derivative(self, t):
        dddxt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t ** 2

        return dddxt


class quartic_polynomial:
    def __init__(self, xs, vxs, axs, vxe, axe, T):
        # 计算四次多项式系数
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * T ** 2, 4 * T ** 3],
                      [6 * T, 12 * T ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * T,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4

        return xt

    def calc_first_derivative(self, t):
        dxt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3

        return dxt

    def calc_second_derivative(self, t):
        ddxt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2

        return ddxt

    def calc_third_derivative(self, t):
        dddxt = 6 * self.a3 + 24 * self.a4 * t

        return dddxt


class FrenetPath:
    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []


def calcFrenetPaths(c_speed, c_d, c_d_d, c_d_dd, s0):
    frenet_paths = []

    # 采样，并对每一个目标配置生成轨迹,给出道路边界,-7-7m,每1m取一个点
    for di in np.arange(BOTTOM_ROAD_WIDTH, TOP_ROAD_WIDTH, D_ROAD_W):

        # 规划预测时间内的轨迹
        for Ti in np.arange(MINT, MAXT, DT):
            fp = FrenetPath()
            # 计算出关于目标配置di，Ti的横向多项式
            #在横向上不考虑di_d,di_dd，因此这两个参数均为0.0,这里求出了5次多项式
            lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # 纵向速度规划 (速度保持) 为什么速度规划用了4次多项式呢?
            for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE, TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S):
                tfp = copy.deepcopy(fp)
                lon_qp = quartic_polynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk
                # square of diff from target speed   
                ds = (TARGET_SPEED - tfp.s_d[-1]) ** 2
                
                # 横向的损失函数
                tfp.cd = KJ * Jp + KT * Ti + KD * tfp.d[-1] ** 2
                # 纵向的损失函数
                tfp.cv = KJ * Js + KT * Ti + KD * ds
                
                # 总的损失函数为d 和 s方向的损失函数乘对应的系数相加
                tfp.cf = KLAT * tfp.cd + KLON * tfp.cv

                frenet_paths.append(tfp)
                
    return frenet_paths


def calcGlobalPaths(fplist, csp):
    for fp in fplist:

        # 计算全局位置
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])
            if ix is None:
                break
            iyaw = csp.calc_yaw(fp.s[i])
            di = fp.d[i]
            fx = ix + di * math.cos(iyaw + math.pi / 2.0)
            fy = iy + di * math.sin(iyaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(math.atan2(dy, dx))
            fp.ds.append(math.sqrt(dx ** 2 + dy ** 2))

        fp.yaw.append(fp.yaw[-1])
        fp.ds.append(fp.ds[-1])

        # calc curvature
        for i in range(len(fp.yaw) - 1):
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

    return fplist


def check_collision(fp, ox, oy, kdtree):
    if collision_check.check_collision(ox, oy, fp.x, fp.y, fp.yaw, kdtree) == False:
        return False
    else:
        return True


def checkPaths(fplist, ox, oy, kdtree):
    okind = []
    for i in range(len(fplist)):
        if any([v > MAX_SPEED for v in fplist[i].s_d]):  # 最大速度检查
            continue
        elif any([abs(a) > MAX_ACCEL for a in fplist[i].s_dd]):  # 最大加速度检查
            continue
        elif any([abs(c) > MAX_CURVATURE for c in fplist[i].c]):  # 最大曲率检查
            continue
        elif not check_collision(fplist[i], ox, oy, kdtree):
            continue

        okind.append(i)

    return [fplist[i] for i in okind]


def frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ox, oy):
    global tox,toy
    ox, oy = ox[:], oy[:]
    tox, toy = ox[:], oy[:]
    kdtree = KDTree(np.vstack((tox, toy)).T)
    fplist = calcFrenetPaths(c_speed, c_d, c_d_d, c_d_dd, s0)
    fplist = calcGlobalPaths(fplist, csp)
    fplist = checkPaths(fplist, ox, oy, kdtree)

    # 找到损失最小的轨迹
    mincost = float("inf")
    bestpath = None
    for fp in fplist:
        if mincost >= fp.cf:
            mincost = fp.cf
            bestpath = fp

    return bestpath


def generateTargetCourse(curX, curY, curYaw, obstacleXList, obstacleYList):
    x = []
    y = []
    curvatureR = [3, 6, 8, 10, 13, 18, 23, 30, 40, 52, 68, 89, 116, 152, 199, 260, 341, 446, 584, 764, 1000]
    maxDistance = 0
    for i in range(len(curvatureR)):
        plannedCircleR = curvatureR[i]
        plannedCircleOx1 = curX + plannedCircleR * math.sin(curYaw)
        plannedCircleOy1 = curY - plannedCircleR * math.cos(curYaw)
        plannedCircleOx2 = curX - plannedCircleR * math.sin(curYaw)
        plannedCircleOy2 = curY + plannedCircleR * math.cos(curYaw)
        theta1 = math.atan2((curY - plannedCircleOy1), (curX - plannedCircleOx1))
        theta2 = math.atan2((curY - plannedCircleOy2), (curX - plannedCircleOx2))
        if theta1 < 0:
            theta1 = theta1 + 2 * math.pi
        if theta2 < 0:
            theta2 = theta2 + 2 * math.pi
        
        clockwisePlannedX = []
        clockwisePlannedY = []
        for j in range(360):
            clockwisePlannedX.append(plannedCircleOx1 + plannedCircleR * math.cos(theta1 - j * math.pi / 180))
            clockwisePlannedY.append(plannedCircleOy1 + plannedCircleR * math.sin(theta1 - j * math.pi / 180))
            
        clockwiseObstaclesX = []
        clockwiseObstaclesY = []
        for j in range(len(obstacleXList)):
            disToCircleO1 = math.sqrt((obstacleXList[j] -  plannedCircleOx1) * (obstacleXList[j] -  plannedCircleOx1) + (obstacleYList[j] -  plannedCircleOy1)*(obstacleYList[j] -  plannedCircleOy1))
            if( (disToCircleO1 - plannedCircleR) > -0.5 and (disToCircleO1 - plannedCircleR) < 0.5):
                clockwiseObstaclesX.append(obstacleXList[j])
                clockwiseObstaclesY.append(obstacleYList[j])
        clockwisePlannedNumber = 360
        for j in range(len(clockwiseObstaclesX)):
            theta = math.atan2((clockwiseObstaclesY[j] - plannedCircleOy1),(clockwiseObstaclesX[j] - plannedCircleOx1))
            if (theta < 0):
                theta = theta + 2 * math.pi
            deltaTheta = theta1 - theta
            if(deltaTheta < 0):
                deltaTheta = deltaTheta + 2 * math.pi;
            tempClockwisePlannedNumber = int(deltaTheta/(math.pi/180))
            if(tempClockwisePlannedNumber < clockwisePlannedNumber):
                clockwisePlannedNumber = tempClockwisePlannedNumber

        antiClockwisePlannedX = []
        antiClockwisePlannedY = []
        for j in range(360):
            antiClockwisePlannedX.append(plannedCircleOx2 + plannedCircleR * math.cos(theta2 + j * math.pi / 180))
            antiClockwisePlannedY.append(plannedCircleOy2 + plannedCircleR * math.sin(theta2 + j * math.pi / 180))
        antiClockwiseObstaclesX = []
        antiClockwiseObstaclesY = []
        for j in range(len(obstacleXList)):
            disToCircleO2 = math.sqrt((obstacleXList[j] -  plannedCircleOx2) * (obstacleXList[j] -  plannedCircleOx2) + (obstacleYList[j] -  plannedCircleOy2)*(obstacleYList[j] -  plannedCircleOy2))
            if( (disToCircleO2 - plannedCircleR) > -0.5 and (disToCircleO2 - plannedCircleR) < 0.5):
                antiClockwiseObstaclesX.append(obstacleXList[j])
                antiClockwiseObstaclesY.append(obstacleYList[j])
        antiClockwisePlannedNumber = 360
        for j in range(len(antiClockwiseObstaclesX)):
            theta = math.atan2((antiClockwiseObstaclesY[j] - plannedCircleOy2),(antiClockwiseObstaclesX[j] - plannedCircleOx2))
            if (theta < 0):
                theta = theta + 2 * math.pi
            deltaTheta = theta - theta2
            if(deltaTheta < 0):
                deltaTheta = deltaTheta + 2 * math.pi
            tempAntiClockwisePlannedNumber = int(deltaTheta/(math.pi/180))
            if(tempAntiClockwisePlannedNumber < antiClockwisePlannedNumber):
                antiClockwisePlannedNumber = tempAntiClockwisePlannedNumber

        tempDis = 0
        if(clockwisePlannedNumber > antiClockwisePlannedNumber):
            for j in range(clockwisePlannedNumber - 1):
                tempDis = tempDis + math.sqrt((clockwisePlannedX[j+1]-clockwisePlannedX[j])*(clockwisePlannedX[j+1]-clockwisePlannedX[j]) + (clockwisePlannedY[j+1]-clockwisePlannedY[j])*(clockwisePlannedY[j+1]-clockwisePlannedY[j]))
            if(tempDis > maxDistance):
                maxDistance = tempDis
                x.clear()
                y.clear()
                for j in range(180):
                    x.append(clockwisePlannedX[j])
                    y.append(clockwisePlannedY[j])
        else:
            for j in range(antiClockwisePlannedNumber - 1):
                tempDis = tempDis + math.sqrt((antiClockwisePlannedX[j+1]-antiClockwisePlannedX[j])*(antiClockwisePlannedX[j+1]-antiClockwisePlannedX[j]) + (antiClockwisePlannedY[j+1]-antiClockwisePlannedY[j])*(antiClockwisePlannedY[j+1]-antiClockwisePlannedY[j]))
            if(tempDis > maxDistance):
                maxDistance = tempDis
                x.clear()
                y.clear()
                for j in range(180):
                    x.append(antiClockwisePlannedX[j])
                    y.append(antiClockwisePlannedY[j])
    csp = cubic_spline.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp

def generateTargetCourseFromBoundaries(x, y):
    csp = cubic_spline.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp

class KDTree:
    """
    Nearest neighbor search class with KDTree
    Dimension is two
    """

    def __init__(self, data):
        # store kd-tree
        self.tree = scipy.spatial.cKDTree(data)

    def search(self, inp, k=1):
        """
        k=1 means to query the nearest neighbours and return squeezed result
        inp: input data
        """

        if len(inp.shape) >= 2:  # multi input 
            index = []
            dist = []
            for i in inp.T:
                idist, iindex = self.tree.query(i, k=k)
                index.append(iindex)
                dist.append(idist)

            return index, dist
        else:
            dist, index = self.tree.query(inp, k=k)
            return index, dist

    def search_in_distance(self, inp, r):
        """
        find points within a distance r
        """
        index = self.tree.query_ball_point(inp, r)
        return index


def main():
    # 路线
#    wx = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0,100.0, 110.0, 120.0]
#    wy = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]

#    wx = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0,100.0, 110.0]
#    wy = [0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0, 22.0]

    # 障碍物列表
    ox = []
    oy = []
    obstacle = []
    boundary1 = []
    boundary2 = []
    

    for i in range(90):
        ox.append(i)
        oy.append(4 + 4 * math.sin(i/20))
        boundary1.append([i, 4 + 4 * math.sin(i/20)])
    for i in range(90):
        ox.append(i)
        oy.append(-4 + 4 * math.sin(i/20))
        boundary2.append([i, -4 + 4 * math.sin(i/20)])
    for i in range(5):
        ox.append(90)
        oy.append(-i)

    for i in range(1):
        ox.append(60)
        oy.append(i)
    

    for (x,y) in obstacle:
        ox.append(x)
        oy.append(y)
        
    
    wx, wy = get_boundaries_centerline.getBoundariesCenterLine(boundary1, boundary2, -10, 0, 0)
    plt.plot(wx,wy)
    curX = -6
    curY = 0
    curYaw = 0

    tx, ty, tyaw, tk, csp = generateTargetCourse(curX, curY, curYaw, ox, oy)

    # 初始状态
    c_speed = 10/3.6  # 当前车速 [m/s]
    c_d = 0  # 当前的d方向位置 [m]
    c_d_d = 0.0  # 当前横向速度 [m/s]
    c_d_dd = 0.0  # 当前横向加速度 [m/s2]
    s0 = 0.0  # 当前所在的位置

    area = 40.0  # 动画显示区间 [m]
#    time_start = time.time()
    
    for i in range(50000):
        path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ox, oy)
#        time_end = time.time()
#        print("during time",time_end - time_start)
        if (path != None and (path.s[-1] - path.s[0]) > 12):
            curX = path.x[1]
            curY = path.y[1]
            curYaw = path.yaw[1]
#            print(lastPathX, lastPathY)
        else:
            tx, ty, tyaw, tk, csp = generateTargetCourse(curX, curY, curYaw, ox, oy)
            c_speed = 20 / 3.6  # 当前车速 [m/s]
            c_d = 0  # 当前的d方向位置 [m]
            c_d_d = 0.0  # 当前横向速度 [m/s]
            c_d_dd = 0.0  #当前横向加速度 [m/s2]
            s0 = 0.0  # 当前所在的位置
            path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ox, oy)
        s0 = path.s[1]
        c_d = path.d[1]
        c_d_d = path.d_d[1]
        c_d_dd = path.d_dd[1]
        c_speed = path.s_d[1]


        if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 2.0:
            print("到达目标")
            break

        plt.cla()
        plt.plot(tx, ty, "r")
        plt.plot(ox, oy, "ob", markersize = 3)
        plt.plot(path.x[1:], path.y[1:], "-og", markersize = 3)
        plt.axis("equal")
        
        plt.plot(path.x[0], path.y[0], "vc")
        plt.xlim(path.x[0] - area, path.x[0] + area)
        plt.ylim(path.y[0] - area/5, path.y[0] + area/5)
        vehicle.plot_trailer(path.x[0], path.y[0], path.yaw[0], 0)
        
        
        font1 = {'family': 'Times New Roman', 'weight': 'normal', 'size': 15}
        plt.xlabel('x(m)',font1)
        plt.ylabel('y(m)',font1)
        plt.legend(['Reference Line','obstacles','path','first point'], prop=font1)
        plt.title("Local planning result",font1)

        plt.grid(True)
        plt.pause(0.001)

#    最后一针break的会有问题
    plt.show()


if __name__ == '__main__':
    main()