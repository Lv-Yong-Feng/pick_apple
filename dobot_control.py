import DobotDllType as dType
import threading
import math


def check_pos(x, y, z):
    L1 = 135
    L2 = 147
    L3 = 150

    L4 = math.sqrt(x ** 2 + y ** 2) - L3
    L5 = math.sqrt(L4 ** 2 + z ** 2)

    if (L4 < 0) or (L5 >= L1 + L2) or (L2 >= L1 + L5): return False

    angle4 = math.atan(z / L4)
    angle5 = math.acos((L1 ** 2 + L5 ** 2 - L2 ** 2) / (2 * L1 * L5))

    if x > 0:
        angle1 = math.atan(y / x)
    elif x == 0:
        if y > 0:
            angle1 = math.pi / 2
        elif y == 0:
            angle1 = 0
        elif y < 0:
            angle1 = -math.pi / 2
    elif x < 0:
        if y > 0:
            angle1 = math.atan(-x / y) + math.pi / 2
        elif y == 0:
            angle1 = -math.pi
        elif y < 0:
            angle1 = math.atan(-x / y) - math.pi / 2

    angle2 = math.pi / 2 - angle4 - angle5
    angle3 = math.asin((L1 * math.cos(angle2) - z) / L2)

    angle1 = angle1 * 180 / math.pi
    angle2 = angle2 * 180 / math.pi
    angle3 = angle3 * 180 / math.pi

    if (angle1 > -105) and (angle1 < 105) and (angle2 > -5) and (angle2 < 90) and (angle3 > 5) and (angle3 < 85):
        return True
    else:
        return False


class DobotControl:

    def __init__(self, port='COMx'):

        self.x_set = 300.0  # mm
        self.y_set = 0.0
        self.z_set = 80.0

        self.claw_pos = 7.2  # open:5.4/close:9

        self.now_posx = 0
        self.now_posy = 0
        self.now_posz = 0

        self.now_angle1 = 0
        self.now_angle2 = 0
        self.now_angle3 = 0

        ###初始化机械臂###
        CON_STR = {
            dType.DobotConnect.DobotConnect_NoError: "DobotConnect_NoError",
            dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
            dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

        self.api = dType.load()

        state = dType.ConnectDobot(self.api, port, 115200)[0]
        print("Connect status:", CON_STR[state])

        if state == dType.DobotConnect.DobotConnect_NoError:
            thread = threading.Thread(target=self.thread_control, daemon=True)
            thread.start()

    def thread_control(self):

        # 清空队列
        dType.SetQueuedCmdForceStopExec(self.api)
        dType.SetQueuedCmdClear(self.api)

        # 设置爪子舵机接口
        dType.SetIOMultiplexing(self.api, address=4, multiplex=2, isQueued=1)
        dType.SetIOPWM(self.api, address=4, frequency=50, dutyCycle=7.2, isQueued=1)  # open-5.4 close-9

        # 设置爪子的末端位置
        dType.SetEndEffectorParams(self.api, 150, 0, 0, isQueued=1)

        # 设置运动参数
        dType.SetHOMEParams(self.api, 330, 0, 0, 0, isQueued=1)
        # dType.SetPTPCoordinateParams(self.api, 1500, 1500, 0, 0, isQueued=1)
        # dType.SetPTPJointParams(self.api, 200, 200, 1200, 800, 1200, 800, 0, 0, isQueued=1)
        dType.SetPTPCommonParams(self.api, 100, 100, isQueued=1)

        # 回零
        lastIndex = dType.SetHOMECmd(self.api, temp=0, isQueued=1)[0]
        dType.SetQueuedCmdStartExec(self.api)
        while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
            dType.dSleep(100)
        print("Dobot Arm Is Ready")

        # 初始位置
        lastIndex = \
            dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVLXYZMode, self.x_set, self.y_set, self.z_set, 0, isQueued=1)[
                0]
        while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
            dType.dSleep(100)

        while True:
            pose = dType.GetPose(self.api)

            self.now_posx = int(pose[0])
            self.now_posy = int(pose[1])
            self.now_posz = int(pose[2])

            self.now_angle1 = int(pose[4])
            self.now_angle2 = int(pose[5])
            self.now_angle3 = int(pose[6])

            # print("posx:"+str(self.now_posx)+" posy:"+str(self.now_posy)+" posz:"+str(self.now_posz)+" angle1:"+str(self.now_angle1)+" angle2:"+str(self.now_angle2)+" angle3:"+str(self.now_angle3))

            dType.dSleep(100)

    def move(self, destpos):
        if check_pos(destpos[0], destpos[1], destpos[2]):
            self.x_set = destpos[0]
            self.y_set = destpos[1]
            self.z_set = destpos[2]

            dType.ClearAllAlarmsState(self.api)

            lastIndex = \
                dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVJXYZMode, self.x_set, self.y_set, self.z_set, 0,
                                isQueued=1)[
                    0]
            while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
                dType.dSleep(100)

    def movedelt(self, deltx, delty, deltz):

        tmpx = self.x_set + deltx
        tmpy = self.y_set + delty
        tmpz = self.z_set + deltz

        if check_pos(tmpx, tmpy, tmpz):
            self.x_set = tmpx
            self.y_set = tmpy
            self.z_set = tmpz

            dType.ClearAllAlarmsState(self.api)

            lastIndex = \
                dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVLXYZMode, self.x_set, self.y_set, self.z_set, 0,
                                isQueued=1)[
                    0]
            while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
                dType.dSleep(100)

    def claw_open(self):
        self.claw_pos = 5.4
        dType.SetIOPWM(self.api, address=4, frequency=50, dutyCycle=self.claw_pos, isQueued=1)

    def claw_close(self):
        self.claw_pos = 9
        dType.SetIOPWM(self.api, address=4, frequency=50, dutyCycle=self.claw_pos, isQueued=1)
