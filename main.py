import numpy as np
import chassis_control
import dobot_control
import realsense_detect
import time
import Crawl_file

cc = chassis_control.ChassisControl(port="COM20")  # 机器人地盘
dc = dobot_control.DobotControl(port='COM22')  # 机器人机械臂
rd = realsense_detect.RealsenseDetect()
cw = Crawl_file.Crawl()

auto_mode = False
stage_1 = False
stage_2 = False
stage_3 = False


def Crawl_fuc():  # 抓取函数
    print("开始抓取...")
    dc.claw_open()
    dc.move(rd.coordinate_dobot)
    dc.claw_close()
    time.sleep(0.15)
    dc.move([0, 300, 120])
    dc.claw_open()
    dc.move([200, 0, 40])
    print("结束抓取...")


count = 0
while True:
    if cc.sw_left == 2:  # 左拨档下,手动模式控制底盘运动
        auto_mode = False
        vx = cc.ch4
        vy = cc.ch3
        vw = cc.ch1
        cc.run(vx, -vy, -vw)
    elif cc.sw_left == 3:  # 左拨档中,手动模式控制机械臂动作
        auto_mode = False
        cc.run(0, 0, 0)
        deltx = cc.ch4 // 10
        delty = -cc.ch3 // 10
        deltz = cc.ch2 // 10
        dc.movedelt(deltx, delty, deltz)
        if cc.wheel > 300:
            dc.claw_open()
        elif cc.wheel < -300:
            dc.claw_close()
    elif cc.sw_left == 1:  # 左拨档上,自动模式
        auto_mode = True
    if auto_mode:  # 自动模式
        if not stage_1:
            to_xv = 660
            to_yv = 0
            to_rw = 0
            cc.run(int(to_xv), int(to_yv), int(to_rw))
            time.sleep(2.6)
            stage_1 = True
        if stage_1 and (stage_2 is False):
            rd_coordinate = rd.coordinate_dobot_green
            print(rd_coordinate)
            rd_coordinate_car = rd.coordinate_car
            to_xv = 0
            to_yv = 0
            to_rw = 0
            pose = False
            if rd_coordinate[0] > 1000:  # x轴
                to_xv = 660  # x移动速度
            else:  # x轴
                to_xv = rd_coordinate[0] * 0.6  # x移动速度
            if abs(rd_coordinate[1]) > 200:  # y轴
                to_yv = 500 * np.sign(rd_coordinate[1])  # y移动速度
            else:
                to_yv = rd_coordinate[1] * 1.3  # y移动速度
            if len(rd_coordinate_car):
                if rd_coordinate[0] > 1000:  # x轴
                    to_yv = 600 * np.sign(rd_coordinate_car[0][1])  # 移动到左端点
                if rd_coordinate_car[0][0] > rd_coordinate_car[1][0]:
                    to_rw = (rd_coordinate_car[1][0] - rd_coordinate_car[0][0]) * 2  # 左右平衡

            if dobot_control.check_pos(rd_coordinate[0] + 15, rd_coordinate[1] - 100, rd_coordinate[2]):  # 检测是否可抓取
                pose = True
            # print('GREEN')
            # print(pose)
            if pose is False and sum(rd_coordinate) == 0:  # 如果没有苹果则右移
                to_yv = -300
            if rd_coordinate[0] < 300:  # x轴
                to_xv = -80  # x移动速度
            # print(to_rw)
            cc.run(int(to_xv), int(to_yv), int(to_rw))
            cw.Judge = 0
            cw.begin = 0
            if pose:  # 到达指定位置
                cc.run(0, 0, 0)
                time.sleep(0.1)
                rd_coordinate = rd.coordinate_dobot_green
                if dobot_control.check_pos(rd_coordinate[0] + 15, rd_coordinate[1] - 100, rd_coordinate[2]):  # 检测是否可抓取
                    cw.destpos = rd_coordinate  # 先传参数
                    cw.Judge = 2
                    print(cw.Judge)
                    while not cw.begin:
                        time.sleep(0.01)
                        # cc.run(0, 0, 0)
                    stage_2 = True
        if stage_2 and (stage_3 is False):
            to_xv = -660
            to_yv = 0
            to_rw = 0
            cw.Judge = 0
            cc.run(int(to_xv), int(to_yv), int(to_rw))      # 后退
            time.sleep(1.4)
            to_xv = -0
            to_yv = -344
            to_rw = 0
            cc.run(int(to_xv), int(to_yv), int(to_rw))
            time.sleep(0.84)
            cc.run(0, 0, 0)
            cw.wait = False
            while not cw.begin:  # 等待机械臂放苹果
                time.sleep(0.01)
            cc.run(510, 660, 0)
            time.sleep(1.6)
            stage_3 = True

        if stage_3:
            rd_coordinate = rd.coordinate_dobot
            rd_coordinate_car = rd.coordinate_car
            # print(rd_coordinate)
            # print(rd_coordinate_car)
            to_xv = 0
            to_yv = 0
            to_rw = 0
            pose = False
            if rd_coordinate[0] > 1000:  # x轴
                to_xv = 600  # x移动速度
            else:  # x轴
                to_xv = rd_coordinate[0] * 0.6  # x移动速度
            if abs(rd_coordinate[1]) > 200:  # y轴
                to_yv = 500 * np.sign(rd_coordinate[1])  # y移动速度
            else:
                to_yv = rd_coordinate[1] * 1.3  # y移动速度
            if len(rd_coordinate_car):
                if rd_coordinate[0] > 1000:  # x轴
                    to_yv = 600 * np.sign(rd_coordinate_car[0][1])  # 移动到左端点
                if rd_coordinate_car[0][0] > rd_coordinate_car[1][0]:
                    to_rw = (rd_coordinate_car[1][0] - rd_coordinate_car[0][0]) * 2  # 左右平衡

            if dobot_control.check_pos(rd_coordinate[0], rd_coordinate[1] - 100, rd_coordinate[2]):  # 检测是否可抓取
                pose = True
            if pose is False and sum(rd_coordinate) == 0:  # 如果没有苹果则右移
                to_yv = -300
            if rd_coordinate[0] < 300:  # x轴
                to_xv = -80  # x移动速度
            # print(to_rw)
            cc.run(int(to_xv), int(to_yv), int(to_rw))
            cw.Judge = 0
            cw.begin = 0
            if pose:  # 到达指定位置
                cc.run(0, 0, 0)
                time.sleep(0.1)
                rd_coordinate = rd.coordinate_dobot
                if dobot_control.check_pos(rd_coordinate[0] + 15, rd_coordinate[1] - 100, rd_coordinate[2]):  # 检测是否可抓取
                    cw.Judge = 1
                    cw.destpos = rd_coordinate
                    while not cw.begin:
                        time.sleep(0.01)

    time.sleep(0.05)  # 50ms
