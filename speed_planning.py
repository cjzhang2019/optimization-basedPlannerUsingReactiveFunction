# -*- coding: utf-8 -*-
"""
Created on Sat Mar 28 10:48:49 2020
@author: 86159
speed planning using S-T graph
"""

import numpy as np
import matplotlib.pyplot as plt
import math
import cubic_spline

font1 = {'family' : 'Times New Roman',
'weight' : 'normal',
'size'   : 20,
}
obList = []
predictTime = 20
predictLength = 40
map_height = 40
map_width = 20
safeDistance = 5
#障碍物平行四边形四点
obstaclePredictionResultList = []
obstaclePredictionResult = {}
refSpeed = 5

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __eq__(self, other): #函数重载
        if((self.x == other.x )and (self.y == other.y)):
            return  1
        else:
            return 0
start_point = Point(0,0)
end_point = Point(39,19)

class map_2d:
    def __init__(self,height,width):#定义栅格地图的长宽
        self.height = height
        self.width = width
        self.data = []
        for i in range(self.height):
            self.data.append([])
            for j in range(self.width):
                self.data[i].append(0)
        
    def map_show(self):
        for i in range(self.height):
            for j in range(self.width):
                if (self.data[i][j] == 5):
                    print('\033[35m%s\033[0m'%('*'), end=' ')
                else:
                    print(self.data[i][j], end=' ')
            print("")
            #打印出结果
    def obstacle(self,obstacle_x,obstacle_y):
        self.data[obstacle_x][obstacle_y] = 2
        
    def expandedObstacle(self,expanded_obstacle_x,expanded_obstacle_y):
        self.data[expanded_obstacle_x][expanded_obstacle_y] = 1
        
    def end_draw(self,point):
        self.data[point.x][point.y] = 5
    #定义障碍物为2，膨胀后障碍物为1，轨迹点为5

class A_star:
    # 设置node
    class Node:
        def __init__(self, point, endpoint, g):
            self.point = point  # 当前位置
            self.endpoint = endpoint  # 终点
            self.father = None  # 父节点
            self.g = g  # g值，g值在用到的时候会重新算
            self.h = (abs(endpoint.x - point.x) + abs(endpoint.y - point.y)) * 10  # 计算h值，曼哈顿距离
            self.f = self.g + self.h

        #寻找临近点
        def search_near(self,ud,rl):  # up  down  right left
            nearpoint = Point(self.point.x + rl, self.point.y + ud)
            nearnode = A_star.Node(nearpoint, self.endpoint, self.g + 1)   # 表示多一步
            return nearnode


    def __init__(self,start_point,end_point,map):
        self.path=[]
        self.close_list=[] #存放已走过的点
        self.open_list=[] #存放需探索的点
        self.current = 0 #现在的node
        self.start_point=start_point
        self.end_point=end_point
        self.map = map #所在地图

    def select_current(self):
        min=10000000
        node_temp = 0
        for ele in self.open_list:
            if ele.f < min:
                min = ele.f
                node_temp = ele
        self.path.append(node_temp)
        self.open_list.remove(node_temp)
        self.close_list.append(node_temp)
        return node_temp

    def isin_openlist(self,node):
        for opennode_temp in self.open_list:
            if opennode_temp.point == node.point:
                return opennode_temp
        return 0

    def isin_closelist(self,node):
        for closenode_temp in self.close_list:
            if closenode_temp.point == node.point:
                return 1
        return 0

    def is_obstacle(self,node):
        if node.point.x < 0 or node.point.x > map_height-1 or node.point.y < 0 or node.point.y > map_width-1:
            return 2
        if (self.map.data[node.point.x][node.point.y] == 2):
            return  2
        elif (self.map.data[node.point.x][node.point.y] == 1):
            return  1
        else:
            return  0
    #这里2表示感知得到的障碍物点，1表示经过膨胀后的障碍物点，2优先于1

    def near_explore(self,node):
        ud = 1
        rl = 0
        node_temp = node.search_near(ud,rl) #在调用另一个类的方法时（不论是子类还是在类外定义的类），都要进行实例化才能调用函数
        if node_temp.point == end_point:
            return 1
        elif self.isin_closelist(node_temp):
            pass
        elif self.is_obstacle(node_temp):
            
            pass
        elif self.isin_openlist(node_temp) == 0:
            node_temp.father = node
            self.open_list.append(node_temp)
        else:
            if node_temp.f < (self.isin_openlist(node_temp)).f:
                self.open_list.remove(self.isin_openlist(node_temp))
                node_temp.father = node
                self.open_list.append(node_temp)

        ud = -1
        rl = 0
        node_temp = node.search_near(ud,rl) #在调用另一个类的方法时（不论是子类还是在类外定义的类），都要进行实例化才能调用函数
        if node_temp.point == end_point:
            return 1
        elif self.isin_closelist(node_temp):
            pass
        elif self.is_obstacle(node_temp):
            pass
        elif self.isin_openlist(node_temp) == 0:
            node_temp.father = node
            self.open_list.append(node_temp)
        else:
            if node_temp.f < (self.isin_openlist(node_temp)).f:
                self.open_list.remove(self.isin_openlist(node_temp))
                node_temp.father = node
                self.open_list.append(node_temp)

        ud = 0
        rl = 1
        node_temp = node.search_near(ud,rl) #在调用另一个类的方法时（不论是子类还是在类外定义的类），都要进行实例化才能调用函数
        if node_temp.point == end_point:
            return 1
        elif self.isin_closelist(node_temp):
            pass
        elif self.is_obstacle(node_temp):
            pass
        elif self.isin_openlist(node_temp) == 0:
            node_temp.father = node
            self.open_list.append(node_temp)
        else:
            if node_temp.f < (self.isin_openlist(node_temp)).f:
                self.open_list.remove(self.isin_openlist(node_temp))
                node_temp.father = node
                self.open_list.append(node_temp)

        ud = 0
        rl = -1
        node_temp = node.search_near(ud,rl) #在调用另一个类的方法时（不论是子类还是在类外定义的类），都要进行实例化才能调用函数
        if node_temp.point == end_point:
            return 1
        elif self.isin_closelist(node_temp):
            pass
        elif self.is_obstacle(node_temp):
            pass
        elif self.isin_openlist(node_temp) == 0:
            node_temp.father = node
            self.open_list.append(node_temp)
        else:
            if node_temp.f < (self.isin_openlist(node_temp)).f:
                self.open_list.remove(self.isin_openlist(node_temp))
                node_temp.father = node
                self.open_list.append(node_temp)

        ud = 1
        rl = 1
        node_temp = node.search_near(ud,rl) #在调用另一个类的方法时（不论是子类还是在类外定义的类），都要进行实例化才能调用函数
        if node_temp.point == end_point:
            return 1
        elif self.isin_closelist(node_temp):
            pass
        elif self.is_obstacle(node_temp):
            pass
        elif self.isin_openlist(node_temp) == 0:
            node_temp.father = node
            self.open_list.append(node_temp)
        else:
            if node_temp.f < (self.isin_openlist(node_temp)).f:
                self.open_list.remove(self.isin_openlist(node_temp))
                node_temp.father = node
                self.open_list.append(node_temp)

        ud = 1
        rl = -1
        node_temp = node.search_near(ud,rl) #在调用另一个类的方法时（不论是子类还是在类外定义的类），都要进行实例化才能调用函数
        if node_temp.point == end_point:
            return 1
        elif self.isin_closelist(node_temp):
            pass
        elif self.is_obstacle(node_temp):
            pass
        elif self.isin_openlist(node_temp) == 0:
            node_temp.father = node
            self.open_list.append(node_temp)
        else:
            if node_temp.f < (self.isin_openlist(node_temp)).f:
                self.open_list.remove(self.isin_openlist(node_temp))
                node_temp.father = node
                self.open_list.append(node_temp)

        ud = -1
        rl = 1
        node_temp = node.search_near(ud,rl) #在调用另一个类的方法时（不论是子类还是在类外定义的类），都要进行实例化才能调用函数
        if node_temp.point == end_point:
            return 1
        elif self.isin_closelist(node_temp):
            pass
        elif self.is_obstacle(node_temp):
            pass
        elif self.isin_openlist(node_temp) == 0:
            node_temp.father = node
            self.open_list.append(node_temp)
        else:
            if node_temp.f < (self.isin_openlist(node_temp)).f:
                self.open_list.remove(self.isin_openlist(node_temp))
                node_temp.father = node
                self.open_list.append(node_temp)

        ud = -1
        rl = -1
        node_temp = node.search_near(ud,rl) #在调用另一个类的方法时（不论是子类还是在类外定义的类），都要进行实例化才能调用函数
        if node_temp.point == end_point:
            return 1
        elif self.isin_closelist(node_temp):
            pass
        elif self.is_obstacle(node_temp):
            pass
        elif self.isin_openlist(node_temp) == 0:
            node_temp.father = node
            self.open_list.append(node_temp)
        else:
            if node_temp.f < (self.isin_openlist(node_temp)).f:
                self.open_list.remove(self.isin_openlist(node_temp))
                node_temp.father = node
                self.open_list.append(node_temp)

        return 0

def generate_target_course(x, y):
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

def obstaclePrediction(obList, tx, ty, predictTime, predictLength):
    x0 = []
    y0 = []
    s = []
    d = []
    t1 = []
    t2 = []
    s1 = []
    s2= []
    s3 = []
    s4 = []
    for i in range(len(obList)):
        x0.append(0)
        y0.append(0)
        s.append(0)
        d.append(0)
        t1.append(0)
        t2.append(0)
        s1.append(0)
        s2.append(0)
        s3.append(0)
        s4.append(0)
    for obNumber in range(len(obList)): 
        for i in range(len(tx)-1):
            a = [math.cos(math.pi*obList[obNumber]['heading']/180), math.sin(math.pi*obList[obNumber]['heading']/180)]
            b = [tx[i]-obList[obNumber]['x'],ty[i]-obList[obNumber]['y']]
            c = [tx[i+1]-obList[obNumber]['x'],ty[i+1]-obList[obNumber]['y']]
            if((a[0]*b[1]-a[1]*b[0])*(a[0]*c[1]-a[1]*c[0]) > 0 or (a[0]*b[0]+a[1]*b[1]) < 0):
                s[obNumber] = s[obNumber] + math.sqrt((tx[i+1] - tx[i]) * (tx[i+1] - tx[i]) + (ty[i+1] - ty[i]) * (ty[i+1] - ty[i]))
            else:
                if(obList[obNumber]['heading'] == 90 or obList[obNumber]['heading'] == -90):
                    x0[obNumber] = obList[obNumber]['x']
                    y0[obNumber] = (ty[i+1]-ty[i])/(tx[i+1]-tx[i])*(x0[obNumber]-tx[i])+ty[i]
                    s[obNumber] = s[obNumber] + math.sqrt((x0[obNumber] - tx[i]) * (x0[obNumber] - tx[i]) + (y0[obNumber] - ty[i]) * (y0[obNumber] - ty[i]))
                else:
                    if(tx[i] == tx[i+1]):
                        x0[obNumber] = tx[i]
                        y0[obNumber] = math.tan(math.pi*obList[obNumber]['heading']/180) * (x0[obNumber]-obList[obNumber]['x']) + obList[obNumber]['y']
                        s[obNumber] = s[obNumber] + math.sqrt((x0[obNumber] - tx[i]) * (x0[obNumber] - tx[i]) + (y0[obNumber] - ty[i]) * (y0[obNumber] - ty[i]))
                    else:  
                        x0[obNumber] = ((math.tan(math.pi*obList[obNumber]['heading']/180)) * obList[obNumber]['x'] + ty[i] - obList[obNumber]['y'] - (ty[i+1]-ty[i])/(tx[i+1]-tx[i]) * tx[i])/((math.tan(math.pi*obList[obNumber]['heading']/180))-(ty[i+1]-ty[i])/(tx[i+1]-tx[i]))
                        y0[obNumber] = math.tan(math.pi*obList[obNumber]['heading']/180) * (x0[obNumber]-obList[obNumber]['x']) + obList[obNumber]['y']
                        s[obNumber] = s[obNumber] + math.sqrt((x0[obNumber] - tx[i]) * (x0[obNumber] - tx[i]) + (y0[obNumber] - ty[i]) * (y0[obNumber] - ty[i]))
                break
    for obNumber in range(len(obList)):
        if(x0[obNumber] == 0 and y0[obNumber] == 0):
            obstaclePredictionResult = {'id':obList[obNumber]['id'],'pointX':[],'pointY':[]}
            obstaclePredictionResultList.append(obstaclePredictionResult)
        else:
            d[obNumber] = math.sqrt((obList[obNumber]['x'] - x0[obNumber]) * (obList[obNumber]['x'] - x0[obNumber]) + (obList[obNumber]['y'] - y0[obNumber]) * (obList[obNumber]['y'] - y0[obNumber]))
            t1[obNumber] = (d[obNumber])/obList[obNumber]['speed']
            t2[obNumber] = t1[obNumber] + 4
            #这里需要好好思考一下，我设置的是预测她在我们道路上10s，但事实应该是会一直在？
            s1[obNumber] = s[obNumber]
            s2[obNumber] = obList[obNumber]['speed']*(t2[obNumber] - t1[obNumber]) + s1[obNumber]
            s3[obNumber] = s2[obNumber] + 5
            s4[obNumber] = s1[obNumber] + 5
            obstaclePredictionResult = {'id':obList[obNumber]['id'],'pointX':[t1[obNumber], t2[obNumber], t2[obNumber], t1[obNumber]],'pointY':[s1[obNumber], s2[obNumber], s3[obNumber], s4[obNumber]],'sDecision':obList[obNumber]['sDecision']}
            obstaclePredictionResultList.append(obstaclePredictionResult)
    return obstaclePredictionResultList

#def velocityPlanning(obstaclePredictionResultList, safeDistance):
#    velocityPointList = []
#    for obNumber in range(len(obstaclePredictionResultList)):
#        if obstaclePredictionResultList[obNumber]['sDecision'] == 0:
#            x1 = obstaclePredictionResultList[obNumber]['pointX'][0]
#            y1 = obstaclePredictionResultList[obNumber]['pointY'][0]-safeDistance
#            if y1 < 0:
#                y1 = 0
#            x2 = obstaclePredictionResultList[obNumber]['pointX'][1]
#            y2 = obstaclePredictionResultList[obNumber]['pointY'][1]-safeDistance
#            if y2 < 0:
#                y2 = 0
#            velocityPointList.append([x1,y1])
#            velocityPointList.append([x2,y2])
#        else:
#            x1 = obstaclePredictionResultList[obNumber]['pointX'][3] - 5
#            y1 = (obstaclePredictionResultList[obNumber]['pointY'][0] + obstaclePredictionResultList[obNumber]['pointY'][3])/2
#            x2 = obstaclePredictionResultList[obNumber]['pointX'][3]
#            y2 = obstaclePredictionResultList[obNumber]['pointY'][3]+safeDistance
#            velocityPointList.append([x1,y1])
#            velocityPointList.append([x2,y2])
#    for i in range(len(velocityPointList)-1):
#        for j in range(len(velocityPointList)-i-1):
#            if (velocityPointList[j][0] > velocityPointList[j+1][0]):
#                velocityPointList[j],velocityPointList[j+1] = velocityPointList[j+1],velocityPointList[j]
#    y3 = predictLength
#    x3 = velocityPointList[-1][0] + (y3 - velocityPointList[-1][1])/5
#    velocityPointList.append([x3,y3])
#    return velocityPointList

def gridMapEstablished(obstaclePredictionResultList, safeDistance):
    ##建图并设立障碍
    ss=map_2d(map_height,map_width)
    for obNumber in range(len(obstaclePredictionResultList)):
        if (obstaclePredictionResultList[obNumber]['sDecision'] == 0):
            x_w1 = math.floor(obstaclePredictionResultList[obNumber]['pointX'][0])
            y_w1 = math.floor(obstaclePredictionResultList[obNumber]['pointY'][0] - safeDistance)
            x_w2 = math.ceil(obstaclePredictionResultList[obNumber]['pointX'][1])
            y_w2 = math.floor(obstaclePredictionResultList[obNumber]['pointY'][1] - safeDistance)
            vector1 = [x_w2 - x_w1, y_w2 - y_w1]
            for i in range(map_height):
                for j in range(x_w1, x_w2 + 1, 1):
                    vector2 = [j - x_w1, i - y_w1]
                    if(vector1[0]*vector2[1]-vector1[1]*vector2[0] >= 0):
                        ss.obstacle(i,j)
        else:
            x_w1 = math.floor(obstaclePredictionResultList[obNumber]['pointX'][3])
            y_w1 = math.ceil(obstaclePredictionResultList[obNumber]['pointY'][3] + safeDistance)
            x_w2 = math.ceil(obstaclePredictionResultList[obNumber]['pointX'][2])
            y_w2 = math.ceil(obstaclePredictionResultList[obNumber]['pointY'][2] + safeDistance)
            vector1 = [x_w2 - x_w1, y_w2 - y_w1]
            for i in range(map_height):
                for j in range(x_w1, x_w2 + 1, 1):
                    vector2 = [j - x_w1, i - y_w1]
                    if(vector1[0]*vector2[1]-vector1[1]*vector2[0] <= 0):
                        ss.obstacle(i,j)
    ss.end_draw(end_point)
    ss.end_draw(start_point)
    a_star = A_star(start_point,end_point,ss)
    start_node = a_star.Node(start_point,end_point,0)
    a_star.open_list.append(start_node)
    flag=0 #到达终点的标志位
    m=0 #步数统计
    
    #进入循环
    while  flag != 1 :
        a_star.current = a_star.select_current()#从openlist中选取一个node
        flag=a_star.near_explore(a_star.current)#对选中的node进行周边探索
        m=m+1
    roughPathX = []
    roughPathY = []

    #画出地图路径
    for node_path in a_star.path:
        ss.end_draw(node_path.point)
        roughPathX.append(node_path.point.y)
        roughPathY.append(node_path.point.x)
    ss.map_show()
    velocityPointX, velocityPointY, velocityPointYaw, velocityPointK, csp = generate_target_course(roughPathX, roughPathY)
    return velocityPointX, velocityPointY

def drawPerceptionResult(obList, tx, ty):
    plt.figure(1)
    plt.plot(tx, ty, "r")
    for obNumber in range(len(obList)):
        plt.plot(obList[obNumber]['x'],obList[obNumber]['y'] , "*")
        plt.quiver(obList[obNumber]['x'], obList[obNumber]['y'], math.cos(math.pi*obList[obNumber]['heading']/180), math.sin(math.pi*obList[obNumber]['heading']/180), color='b', width=0.005)
    plt.axis('equal')
    plt.xlabel('x(m)',font1)
    plt.ylabel('y(m)',font1)
    plt.show()

def drawObstaclePredictionResult(obstaclePredictionResultList):
    plt.figure(2)
    plt.xlim(0,predictTime)
    plt.ylim(0,predictLength)
    plt.xlabel('t(s)',font1)
    plt.ylabel('s(m)',font1)
    for obNumber in range(len(obstaclePredictionResultList)):
        plt.fill(obstaclePredictionResultList[obNumber]['pointX'],obstaclePredictionResultList[obNumber]['pointY'], facecolor='r')
    plt.show()
    
def drawVelocityPlanningResult(velocityPointX, velocityPointY, obstaclePredictionResultList):
    plt.figure(3)
    plt.xlim(0,predictTime)
    plt.ylim(0,predictLength)
    for obNumber in range(len(obstaclePredictionResultList)):
        plt.fill(obstaclePredictionResultList[obNumber]['pointX'],obstaclePredictionResultList[obNumber]['pointY'], facecolor='r')
    plt.plot(velocityPointX, velocityPointY)
    plt.xlabel('t/(s)',font1)
    plt.ylabel('s/(m)',font1)
    plt.plot(velocityPointX, velocityPointY,'r')
    plt.title('speed planning result', font1)
    plt.show()
    

def main():
    wx = [0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0]
    wy = [0.0, 0.5, 0.0, -0.5, 0.0, 0.0, 0.0, 0.3, 0.6, 0.0, 0.0]
    ob1 = {'id':1,'x':6,'y':-8,'speed':4,'heading':60, 'sDecision':0}
    ob2 = {'id':2,'x':15,'y':-10,'speed':1,'heading':90, 'sDecision':1}
#    ob3 = {'id':3,'x':50,'y':12,'speed':1,'heading':-30, 'sDecision':0}
#here is the output of decision module, 0 means following, 1 means overtaking
    obList.append(ob1)
    obList.append(ob2)
#    obList.append(ob3)
    tx, ty, tyaw, tk, csp = generate_target_course(wx, wy)
    obstaclePredictionResultList = obstaclePrediction(obList, tx, ty, predictTime, predictLength)
    drawPerceptionResult(obList, tx, ty)
    drawObstaclePredictionResult(obstaclePredictionResultList)
    velocityPointX, velocityPointY = gridMapEstablished(obstaclePredictionResultList, safeDistance)
    drawVelocityPlanningResult(velocityPointX, velocityPointY, obstaclePredictionResultList)
    
    
if __name__ == '__main__':
    main()