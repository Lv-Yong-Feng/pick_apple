import numpy as np
import chassis_control
import dobot_control
import realsense_detect
import time
import Crawl_file

crawl = Crawl_file.Crawl()
realsense = realsense_detect.RealsenseDetect(True)
chassis = chassis_control.ChassisControl(port="COM20")  # 机器人地盘
dobot = dobot_control.DobotControl(port='COM22')  # 机器人机械臂

auto_mode = False
stage_1 = False
stage_2 = False
stage_3 = False


def Crawl_fuc():  # 抓取函数
    print("开始抓取...")
    dobot.claw_open()
    dobot.move(realsense.coordinate_dobot)
    dobot.claw_close()
    time.sleep(0.15)
    dobot.move([0, 300, 120])
    dobot.claw_open()
    dobot.move([200, 0, 40])
    print("结束抓取...")


count = 0
while True:
    if chassis.sw_left == 2:  # 左拨档下,手动模式控制底盘运动
        auto_mode = False
        vx = chassis.ch4
        vy = chassis.ch3
        vw = chassis.ch1
        chassis.run(vx, -vy, -vw)
    elif chassis.sw_left == 3:  # 左拨档中,手动模式控制机械臂动作
        auto_mode = False
        chassis.run(0, 0, 0)
        deltx = chassis.ch4 // 10
        delty = -chassis.ch3 // 10
        deltz = chassis.ch2 // 10
        dobot.movedelt(deltx, delty, deltz)
        if chassis.wheel > 300:
            dobot.claw_open()
        elif chassis.wheel < -300:
            dobot.claw_close()
    elif chassis.sw_left == 1:  # 左拨档上,自动模式
        auto_mode = True
    if auto_mode:  # 自动模式
        if not stage_1:
            to_xv = 660
            to_yv = 0
            to_rw = 0
            chassis.run(int(to_xv), int(to_yv), int(to_rw))
            time.sleep(2.6)
            stage_1 = True
        if stage_1 and (stage_2 is False):
            rd_coordinate = realsense.coordinate_dobot
            to_xv = 0
            to_yv = 0
            to_rw = 0
            pose = False

            if rd_coordinate[0] > 1000:      # x轴
                to_xv = 660
            else:
                to_xv = rd_coordinate[0] * 0.6

            if abs(rd_coordinate[1]) > 200:  # y轴
                to_yv = 500 * np.sign(rd_coordinate[1])
            else:
                to_yv = rd_coordinate[1] * 1.3

            if dobot_control.check_pos(rd_coordinate[0] + 15, rd_coordinate[1] - 100, rd_coordinate[2]):  # 检测是否可抓取
                pose = True
            if pose is False and sum(rd_coordinate) == 0:  # 如果没有苹果则右移
                to_yv = -300
            if rd_coordinate[0] < 300:          # x轴
                to_xv = -80                     # x移动速度
            chassis.run(int(to_xv), int(to_yv), int(to_rw))
            crawl.Judge = 0
            crawl.begin = 0

            if pose:                            # 到达指定位置
                chassis.run(0, 0, 0)
                time.sleep(0.1)
                if dobot_control.check_pos(rd_coordinate[0] + 15, rd_coordinate[1] - 100, rd_coordinate[2]):  # 检测是否可抓取
                    crawl.destpos = rd_coordinate  # 先传参数
                    crawl.Judge = 2
                    print(crawl.Judge)
                    while not crawl.begin:
                        time.sleep(0.01)
                        # cc.run(0, 0, 0)
                    stage_2 = True
        if stage_2 and (stage_3 is False):
            to_xv = -660
            to_yv = 0
            to_rw = 0
            crawl.Judge = 0
            chassis.run(int(to_xv), int(to_yv), int(to_rw))      # 后退
            time.sleep(1.4)
            to_xv = -0
            to_yv = -344
            to_rw = 0
            chassis.run(int(to_xv), int(to_yv), int(to_rw))
            time.sleep(0.84)
            chassis.run(0, 0, 0)
            crawl.wait = False
            while not crawl.begin:  # 等待机械臂放苹果
                time.sleep(0.01)
            chassis.run(510, 660, 0)
            time.sleep(1.6)


    time.sleep(0.05)  # 50ms
