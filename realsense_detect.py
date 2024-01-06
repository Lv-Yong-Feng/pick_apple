import dobot_control
import pyrealsense2 as rs
import numpy as np
from pyzbar.pyzbar import decode
import cv2
import time

colors = [(255, 255, 0), (0, 255, 0), (0, 255, 255), (255, 0, 0)]


class RealsenseDetect:

    def __init__(self, is_cuda):

        self.fps = 0.0
        # TODO: coordinate_dobot_list:Type->list[dict]
        # coordinate_dobot:Type->dict
        self.coordinate_dobot_list = []
        self.coordinate_dobot = [0, 0, 0]           # 目标相对于机械臂的坐标
        print("load yolo model, waiting...")
        self.model = self.build_model(is_cuda)      # 模型载入
        # self.thread_realsense_detecting()
        self.thread_realsense_detecting()

    def build_model(self, is_cuda):
        net = cv2.dnn.readNet("./weights/best.onnx")
        if is_cuda:
            print("Attempty to use CUDA")
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
        else:
            print("Running on CPU")
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        return net

    def detect(self, image, net):
        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (640, 640), swapRB=True, crop=False)
        net.setInput(blob)
        preds = net.forward()
        return preds

    def wrap_detection(self, input_image, output_data):
        class_ids = []
        confidences = []
        boxes = []

        rows = output_data.shape[0]

        image_width, image_height, _ = input_image.shape

        x_factor = image_width / 640
        y_factor = image_height / 640

        for r in range(rows):
            row = output_data[r]
            confidence = row[4]
            if confidence >= 0.4:

                classes_scores = row[5:]
                _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
                class_id = max_indx[1]
                if (classes_scores[class_id] > .25):
                    confidences.append(confidence)

                    class_ids.append(class_id)

                    x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item()
                    left = int((x - 0.5 * w) * x_factor)
                    top = int((y - 0.5 * h) * y_factor)
                    width = int(w * x_factor)
                    height = int(h * y_factor)
                    box = np.array([left, top, width, height])
                    boxes.append(box)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.25, 0.45)

        result_class_ids = []
        result_confidences = []
        result_boxes = []

        for i in indexes:
            result_confidences.append(confidences[i])
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])

        return result_class_ids, result_confidences, result_boxes

    def reset_(self):
        self.coordinate_dobot = []

    def thread_realsense_detecting(self):

        context = rs.context()
        devices = context.query_devices()
        print("device_num: ", devices.size())

        rs_device = devices[0]
        print("Name: ", rs_device.get_info(rs.camera_info.name))
        print("product_line: ", rs_device.get_info(rs.camera_info.product_line))
        print("SN: ", rs_device.get_info(rs.camera_info.serial_number))
        print("asicSN: ", rs_device.get_info(rs.camera_info.asic_serial_number))
        print("firmware_version: ", rs_device.get_info(rs.camera_info.firmware_version))
        print("usb_type: ", rs_device.get_info(rs.camera_info.usb_type_descriptor))
        print("advanced_mode: ", rs_device.get_info(rs.camera_info.advanced_mode))

        rs_sensors = rs_device.query_sensors()
        for sensor in rs_sensors:
            print("Sensor " + sensor.get_info(rs.camera_info.name))

        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        pipeline = rs.pipeline()
        pipeline.start(config)

        colorizer = rs.colorizer()
        colorizer.set_option(rs.option.color_scheme, 0)  # 0-Jet 3-BlackToWhite
        colorizer.set_option(rs.option.min_distance, 0)
        colorizer.set_option(rs.option.max_distance, 16)
        colorizer.set_option(rs.option.histogram_equalization_enabled, 1)

        align = rs.align(rs.stream.color)

        while True:
            start_t = time.time()

            self.reset_()

            frames = pipeline.wait_for_frames()
            frames = align.process(frames)

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            depth_color_frame = colorizer.colorize(depth_frame)

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_color_frame.get_data())

            gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            resized_image = cv2.resize(color_image, (640, 640))
            results = self.detect(resized_image, self.model)
            class_ids, confidences, boxs = self.wrap_detection(resized_image, results[0])
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

            for box, class_id in zip(boxs, class_ids):
                box[1] *= 0.75
                box[3] *= 0.75

                # TODO:修改识别逻辑，避免识别多个结果或检测结果与识别结果不匹配
                # 使用box限制decode识别区域,另外将box于识别结果通过字典存储, 字典格式{"label":box}
                decoded_objects = decode(gray_image)

                if len(decoded_objects) != 0:
                    data = decoded_objects[0].data.decode('utf-8')
                    print(data)                                                                 # 二维码扫描结果

                center_point = [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2]                   # 确定中心像素位置
                dist = depth_frame.get_distance(int(center_point[0]), int(center_point[1]))     # 获取该像素点对应的深度

                if dist != 0:
                    coordinate_rs = rs.rs2_deproject_pixel_to_point(depth_intrin, center_point, dist)  # 转换到三维坐标
                    coordinate_rs = [i * 1000 for i in coordinate_rs]                           # 转换到mm

                    coor = [int(coordinate_rs[2] + 130), int(-coordinate_rs[0]),
                            int(-coordinate_rs[1] - 42)]                                        # 转换到机械臂坐标
                    self.coordinate_dobot_list.append(coor)                                     # 全部目标相对机械臂坐标
                    if dobot_control.check_pos(coor[0], coor[1], coor[2]):                      # 超距评估
                        cv2.rectangle(color_image, box, (0, 255, 0), 3)
                    else:
                        cv2.rectangle(color_image, box, (0, 0, 255), 3)
                    cv2.putText(color_image, str(dist)[:5] + 'm', (int(box[0]), int(box[1]) - 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)

            if len(self.coordinate_dobot_list) != 0:
                item_min = self.coordinate_dobot_list[0][0] ** 2 + self.coordinate_dobot_list[0][1] ** 2 + \
                           self.coordinate_dobot_list[0][2] ** 2                                # 判断欧式距离最近苹果
                tag = 0
                i = 0
                for item in self.coordinate_dobot_list[1:]:
                    i += 1
                    if item[0] ** 2 + item[1] ** 2 + item[2] ** 2 < item_min:
                        item_min = item[0] ** 2 + item[1] ** 2 + item[2] ** 2
                        tag = i
                self.coordinate_dobot = self.coordinate_dobot_list[tag]         # 选择最近距离目标
            else:
                self.coordinate_dobot = [0, 0, 0]
            images = np.hstack((color_image, depth_image))
            cv2.putText(images, "fps: {:.2f}".format(self.fps), (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                        (255, 255, 255), 2)

            cv2.imshow("realsense_images", images)
            cv2.waitKey(1)

            end_t = time.time()
            self.fps = 1 / (end_t - start_t)

if __name__ == '__main__':
    rs = RealsenseDetect(True)
