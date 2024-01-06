import threading
import time
import dobot_control
import realsense_detect

dc = dobot_control.DobotControl(port='COM14')  # 机器人机械臂


class Crawl:  # 抓取函数
    def __init__(self):
        self.Judge = 0
        self.destpos = [0, 0, 0]
        self.begin = 0
        self.wait = True
        thread = threading.Thread(target=self.threading_crawl, daemon=True)
        thread.start()
        dc.move([280, 0, 40])
        dc.claw_open()
        time.sleep(0.05)  # 50ms

    def threading_crawl(self):
        while True:
            if self.Judge == 1:
                print("开始抓取...")
                dc.move(self.destpos)
                dc.claw_close()
                time.sleep(0.2)
                self.begin = 1
                dc.move([0, 300, 120])
                dc.claw_open()
                dc.move([280, 0, 40])
                print("结束抓取...")
            if self.Judge == 2:  # green apple

                dc.move(self.destpos)
                dc.claw_close()
                # 完成绿色苹果抓取
                time.sleep(0.2)
                self.begin = 1
                dc.move([0, -360, 30])
                self.begin = 0
                # 等待扫空剩余红苹果并来到回收点
                while self.wait:
                    time.sleep(0.03)
                dc.claw_open()
                self.begin = 1
                time.sleep(0.1)
                dc.move([280, 0, 40])
            time.sleep(0.05)  # 50ms
