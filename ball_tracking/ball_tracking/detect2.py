#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import argparse
import csv
import math
import os
import platform
import sys
from pathlib import Path
import math
import torch
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


current_directory = os.path.dirname(os.path.abspath(__file__))

# Define the relative path to the model file
MODEL_RELATIVE_PATH = "weights/redballbest.pt"

# Combine the current directory and the relative path to get the absolute path
redblue_model_path = os.path.join(current_directory, MODEL_RELATIVE_PATH)
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from ultralytics.utils.plotting import Annotator, colors, save_one_box

from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams
from utils.general import (
    LOGGER,
    Profile,
    check_file,
    check_img_size,
    check_imshow,
    check_requirements,
    colorstr,
    cv2,
    increment_path,
    non_max_suppression,
    print_args,
    scale_boxes,
    strip_optimizer,
    xyxy2xywh,
)
from utils.torch_utils import select_device, smart_inference_mode


class YOLOv5ROS2(Node):
    
    def __init__(self):
        super().__init__('yolov5_ros2_node')
        self.declare_parameter("setupareaball", -41000.0)       # The desired sensor reading
        self.declare_parameter("setupdev", 120.01)                  # Proportional gain
        self.declare_parameter("setupareasilo", -90000.00)                  # Integral gain
        self.setupareaball = self.get_parameter("setupareaball").value
        self.setupdev = self.get_parameter("setupdev").value
        self.setupareasilo = self.get_parameter("setupareasilo").value
        # self.declare_parameter("Kd", 0.00)
        # Publisher for publishing area and deviation
        self.subscription = self.create_subscription(Image, 'video_frames', self.callback, 10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Twist,'/ball_data',10)
    def callback(self, msg):
        try:
            frame1 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # cv2.imshow("Video Stream", frame)
            # cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error("Error converting image: %s" % str(e))    
        # self.subscription = self.create_subscription(
        #   Int16MultiArray, 
        #   'drive_topic', 
        #   self.listener_callback, 
        #   10)
        # self.subscription # prevent unused variable warning

        # self.array = Int16MultiArray()
        # Timer to periodically run YOLOv5 inference
        # self.timer_ = self.create_timer(1.0, self.inference_callback)

        self.run(weights=redblue_model_path, source=frame1)

        # Initialize YOLOv5 model
        # self.initialize_yolov5_model()    

    @smart_inference_mode()
    def run(
        self,
        weights=redblue_model_path,  # model path or triton URL
        source=0,  # file/dir/URL/glob/screen/0(webcam)
        data=ROOT / "data/coco128.yaml",  # dataset.yaml path
        imgsz=(640, 640),  # inference size (height, width)
        conf_thres=0.7,  # confidence threshold
        iou_thres=0.45,  # NMS IOU threshold
        max_det=1000,  # maximum detections per image
        device="cpu",  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        view_img=False,  # show results
        save_txt=False,  # save results to *.txt
        save_csv=False,  # save results in CSV format
        save_conf=False,  # save confidences in --save-txt labels
        save_crop=False,  # save cropped prediction boxes
        nosave=False,  # do not save images/videos
        classes=None,  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms=False,  # class-agnostic NMS
        augment=False,  # augmented inference
        visualize=False,  # visualize features
        update=False,  # update all models
        project=ROOT / "runs/detect",  # save results to project/name
        name="",  # save results to project/name
        exist_ok=False,  # existing project/name ok, do not increment
        line_thickness=3,  # bounding box thickness (pixels)
        hide_labels=False,  # hide labels
        hide_conf=False,  # hide confidences
        half=False,  # use FP16 half-precision inference
        dnn=False,  # use OpenCV DNN for ONNX inference
        vid_stride=1,  # video frame-rate stride
    ):  
        source = str(source)
        save_img = not nosave and not source.endswith(".txt")  # save inference images
        is_file = Path(source).suffix[1:] in (IMG_FORMATS + VID_FORMATS)
        is_url = source.lower().startswith(("rtsp://", "rtmp://", "http://", "https://"))
        webcam = source.isnumeric() or source.endswith(".streams") or (is_url and not is_file)
        screenshot = source.lower().startswith("screen")
        if is_url and is_file:
            source = check_file(source)  # download

        # Directories
        save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)  # increment run
        (save_dir / "labels" if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

        # Load model
        device = select_device(device)
        model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
        stride, names, pt = model.stride, model.names, model.pt
        imgsz = check_img_size(imgsz, s=stride)  # check image size

        # Dataloader
        bs = 1  # batch_size
        if webcam:
            view_img = check_imshow(warn=True)
            dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt, vid_stride=vid_stride)
            bs = len(dataset)
        elif screenshot:
            dataset = LoadScreenshots(source, img_size=imgsz, stride=stride, auto=pt)
        else:
            dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt, vid_stride=vid_stride)
        vid_path, vid_writer = [None] * bs, [None] * bs

        # Run inference
        model.warmup(imgsz=(1 if pt or model.triton else bs, 3, *imgsz))  # warmup
        seen, windows, dt = 0, [], (Profile(device=device), Profile(device=device), Profile(device=device))
        area_sum = 0
        deviation_sum = 0
        ballfound = False
        LinXb=0
        AngZpb=0
        for path, im, im0s, vid_cap, s in dataset:
            with dt[0]:
                im = torch.from_numpy(im).to(model.device)
                im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
                im /= 255  # 0 - 255 to 0.0 - 1.0
                if len(im.shape) == 3:
                    im = im[None]  # expand for batch dim
                if model.xml and im.shape[0] > 1:
                    ims = torch.chunk(im, im.shape[0], 0)

            # Inference
            with dt[1]:
                visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
                if model.xml and im.shape[0] > 1:
                    pred = None
                    for image in ims:
                        if pred is None:
                            pred = model(image, augment=augment, visualize=visualize).unsqueeze(0)
                        else:
                            pred = torch.cat((pred, model(image, augment=augment, visualize=visualize).unsqueeze(0)), dim=0)
                    pred = [pred, None]
                else:
                    pred = model(im, augment=augment, visualize=visualize)
            # NMS
            with dt[2]:
                pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
            twist_msg = Twist()
            # twist_msg.angular.z = 0.0
            # Process predictions
            
            for i, det in enumerate(pred):  # per image
                seen += 1
                if webcam:  # batch_size >= 1
                    p, im0, frame = path[i], im0s[i].copy(), dataset.count
                    s += f"{i}: "
                else:
                    p, im0, frame = path, im0s.copy(), getattr(dataset, "frame", 0)

                p = Path(p)  # to Path
                save_path = str(save_dir / p.name)  # im.jpg
                txt_path = str(save_dir / "labels" / p.stem) + ("" if dataset.mode == "image" else f"_{frame}")  # im.txt
                s += "%gx%g " % im.shape[2:]  # print string
                gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
                imc = im0.copy() if save_crop else im0  # for save_crop
                annotator = Annotator(im0, line_width=line_thickness, example=str(names))
                if len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

                    # Print results
                    for c in det[:, 5].unique():
                        n = (det[:, 5] == c).sum()  # detections per class
                        s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                    # Print class, area, and deviation
                    for *xyxy, conf, cls in reversed(det):
                        c = int(cls)  # integer class
                        label = names[c] if hide_conf else f"{names[c]}"
                        confidence = float(conf)
                        confidence_str = f"{confidence:.2f}"
                        def map(x, in_min, in_max, out_min, out_max):
                        # Scale the input x from the input range to the output range
                            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
                    
                    # Usage example:
                        # LinX = map(area1, -65000, 0, 0, 1)
                        # AngZp = map(deviation, -230, 230, -6, 6)
                        # Calculate area and deviation
                        x1, y1, x2, y2 = xyxy
                        detections_ball = []
                        detections_silo = []
                        area = abs((x2 - x1) * (y2 - y1))
                        # if label == "Red-ball" or label=="blue-ball":
                        #     detections_ball.append((label, area, deviation, x1, y1, x2, y2))
                        # if label == "silo":
                        #     detections_silo.append((label, area, deviation, x1, y1, x2, y2))
                        # Calculate deviation from the center of the image
                        center_x = im0.shape[1] // 2  # Calculate the center x-coordinate of the image
                        deviation = center_x - ((x1 + x2) // 2)  # Calculate deviation from the center
                        area1=-area
                        # Publish area and deviation on cmd_vel topic
                        
                        # if label=="Red-ball":
                        # detections_silo[0][1]=69
                        area=-area
                        if label == "Redball" or label=="blue-ball":
                            detections_ball.append((label, area, deviation, x1, y1, x2, y2))
                        if label == "silo":
                            print("hi 2")
                            detections_silo.append((label, area, deviation, x1, y1, x2, y2))
                            # print(detections_silo[0][1])
                        #     twist_msg.linear.x = float(area1)
                        #     twist_msg.angular.z = float(deviation)
                        # else:
                        #     twist_msg.linear.x = 0.0
                        #     twist_msg.angular.z = 0.0
                        # if float(area) <=65000:
                        #     twist_msg.linear.x = 1.0
                        # else:
                        #     twist_msg.linear.x = 0.0
                        # deviation1=deviation
                        # self.setupareaball =-40000    
                        # a = 90 # IMU given  Angle 
                        # a= a * 0.01745329251
                        # # area=40000
                        # y= math.cos(a) * area
                        # print(y)

                        deviation=-deviation   
                        AngZpb = map(deviation, -250,self.setupdev, 1, 0)
                        LinXb=map(area, -60000,-1000, 0, 2)
                        LinXs=map(area,-70000,100, 0, 1)
                        # LinZsy=map(y,self.setupareasilo,0,1,0)
                        ballfound=True
                                                                         
                        if label == "Redball" and area>= self.setupareaball:
                            # twist_msg.linear.x = float(LinXb)
                            # twist_msg.angular.z = float(AngZpb)
                            # self.publisher_.publish(twist_msg)
                            # if deviation >=-200 and deviation <=200:
                            twist_msg.linear.x = float(LinXs)
                            twist_msg.angular.z = float(AngZpb)
                            twist_msg.angular.z = 1.0
                            self.publisher_.publish(twist_msg)
                            # twist_msg.linear.y=float(LinZsy)
                            # if (deviation >=-250 and deviation <-200) or (deviation <=250 and deviation >=200)   :
                            #     twist_msg.linear.x = float(LinXs)
                            #     twist_msg.angular.z = float(AngZpb)                                                                                                                                                                                                                                                       
                            #     self.publisher_.publish(twist_msg)
                        # if len(detections_ball) > 0 and detections_ball[0][1] <= self.setupareaball and detections_ball[0][0] == "Red-ball":
                        #     # Robot holds the ball, move towards the silo
                        #     print("hi")

                        #     if label=="silo":
                        #         print("Moving towards silo while holding the ball")
                        #         # Calculate LinXs and AngZpb using your map function based on silo detection
                              
                        #         twist_msg.linear.x = float(LinXs)
                        #         twist_msg.angular.z = float(AngZpb)
                        # if  label=="":    
                        # twist_msg.angular.z = 1.0
                        # if label=="Red-ball" and detections_ball[0][1]>=self.setupareaball:
                        #     twist_msg.linear.x = float(LinXb)
                        #     twist_msg.angular.z = float(AngZpb)
                        
                        if label=="Redball" and area<= self.setupareaball:
                            twist_msg.linear.x = 0.0
                            twist_msg.angular.z = 0.0
                            self.publisher_.publish(twist_msg)
                            twist_msg.linear.x = 0.0
                            twist_msg.angular.z = 0.0
                            self.publisher_.publish(twist_msg)
                            twist_msg.linear.x = 0.0
                            twist_msg.angular.z = 0.0
                            self.publisher_.publish(twist_msg)
                            twist_msg.linear.x = 0.0
                            twist_msg.angular.z = 0.0
                            self.publisher_.publish(twist_msg)
                            twist_msg.linear.x = 0.0
                            twist_msg.angular.z = 0.0
                            self.publisher_.publish(twist_msg)
                            twist_msg.linear.x = 0.0
                            twist_msg.angular.z = 0.0
                            self.publisher_.publish(twist_msg)
                            twist_msg.linear.x = 0.0
                            twist_msg.angular.z = 0.0
                            self.publisher_.publish(twist_msg)
                        #     if  twist_msg.angular.z == 0.0   and twist_msg.linear.x == 0.0:
                        #         cou
                        # # # r = math.sqrt(area/3.14)
                            # count = count +1
                            # print(count)
                            # if count > 5:
                            self.destroy_node()
                        


                            

                        
                        # self.get_logger().info("Enter in node of publisher")
                        
                        # Print class, area, and deviation
                        print(f"Class: {label}, Area: {area}, Deviation: {deviation}")
                        print(ballfound)

                        # if vid_path[i] != save_path:  # new video
                        #     vid_path[i] = save_path
                        #     if isinstance(vid_writer[i], cv2.VideoWriter):
                        #         vid_writer[i].release()  # release previous video writer
                        #     if vid_cap:  # video
                        #         fps = vid_cap.get(cv2.CAP_PROP_FPS)
                        #         w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        #         h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        #     else:  # stream
                        #         fps, w, h = 30, im0.shape[1], im0.shape[0]
                        #     save_path = str(Path(save_path).with_suffix(".mp4"))  # force *.mp4 suffix on results videos
                        #     vid_writer[i] = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
                        # vid_writer[i].write(im0)

                        # if save_txt:  # Write to file
                        #     xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        #     line = (cls, *xywh, conf) if save_conf else (cls, *xywh)  # label format
                        #     with open(f"{txt_path}.txt", "a") as f:
                        #         f.write(("%g " * len(line)).rstrip() % line + "\n")

                        if save_img or save_crop or view_img:  # Add bbox to image
                            c = int(cls)  # integer class
                            label = None if hide_labels else (names[c] if hide_conf else f"{names[c]} {conf:.2f}")
                            annotator.box_label(xyxy, label, color=colors(c, True))
                        if save_crop:
                            save_one_box(xyxy, imc, file=save_dir / "crops" / names[c] / f"{p.stem}.jpg", BGR=True)
                elif ballfound == False :
                    # No objects detected, set linear and angular velocities to zero
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = 0.5
                    print(ballfound)
                    self.publisher_.publish(twist_msg)        
                # Stream results
                elif ballfound == True:
                    print(ballfound)
                    # twist_msg.linear.x = float(LinXb)
                    twist_msg.angular.z = float(AngZpb)
                    self.publisher_.publish(twist_msg)
                im0 = annotator.result()
                if view_img:
                    if platform.system() == "Linux" and p not in windows:
                        windows.append(p)
                        cv2.namedWindow(str(p), cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # allow window resize (Linux)
                        cv2.resizeWindow(str(p), im0.shape[1], im0.shape[0])
                    cv2.imshow(str(p), im0)
                    cv2.waitKey(1)  # 1 millisecond 
        # Return average area and deviation
        # return area_sum, deviation_sum


        # Run YOLOv5 inference
        # Use self.area and self.deviation here
        # area = self.area
        # deviation = self.deviation
        
        # Publish area and deviation on cmd_vel topic






def main(args=None):
    """Executes YOLOv5 model inference with manually specified options."""
    check_requirements(ROOT / "requirements.txt", exclude=("tensorboard ", "thop"))
    # area, deviation = run()  # Capture returned area and deviation values
    rclpy.init(args=args)

    yolov5_node = YOLOv5ROS2()
    
    rclpy.spin(yolov5_node)

    yolov5_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
