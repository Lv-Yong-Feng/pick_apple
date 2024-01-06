import numpy as np
import cv2 as cv
import os

# 1.获取视频对象
save_folder = "data"
cap = cv.VideoCapture(1)


current_frame = 0

# 2.判断是否读取成功
while cap.isOpened():
    # 3.获取每一帧图像
    ret, frame = cap.read()
    current_frame += 1
    inter = 25
    # 4. 获取成功显示图像
    if ret == True:
        cv.imshow('frame', frame)

    if current_frame % inter == 0:
        save_path = os.path.join(save_folder, f"{current_frame // inter + 1}.jpg")
        cv.imwrite(save_path, frame)
    # 5.每一帧间隔为25ms
    if cv.waitKey(25) & 0xFF == ord('q'):
        break
# 6.释放视频对象
cap.release()
