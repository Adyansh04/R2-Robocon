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
current_directory = os.path.dirname(os.path.abspath(__file__))

# Define the relative path to the model file
MODEL_RELATIVE_PATH = "weights/redbluesilo3.pt"

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
        self.declare_parameter("setupdev", 80.01)                  # Proportional gain
        self.declare_parameter("setupareasilo", -15000.00)                  # Integral gain
        self.declare_parameter("Ball_Name", "silo")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('linear_x_max', 2.0),
                ('linear_y_max', 2.0),
                ('linear_x_min', -2.0),
                ('linear_y_min', -2.0),
                ('angular_z_max', 1.0),
                ('angular_z_min', -1.0),
                ('kp_linear', 0.05),
                ('ki_linear', 0.0),
                ('kd_linear', 0.0),
                ('kp_angular', 0.08),
                ('ki_angular', 0.0),
                ('kd_angular', 0.0),                       
                ]
        )

        self.setupareaball = self.get_parameter("setupareaball").value
        self.setupdev = self.get_parameter("setupdev").value
        self.setupareasilo = self.get_parameter("setupareasilo").value
        self.setupballname= self.get_parameter("Ball_Name").value
        self. linear_x_max = self.get_parameter('linear_x_max').value
        self.linear_y_max = self.get_parameter('linear_y_max').value
        self.linear_x_min = self.get_parameter('linear_x_min').value
        self.linear_y_min = self.get_parameter('linear_y_min').value
        self.angular_z_max = self.get_parameter('angular_z_max').value
        self.angular_z_min = self.get_parameter('angular_z_min').value

        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.kd_linear = self.get_parameter('kd_linear').value
        self.ki_angular = self.get_parameter('kd_angular').value
        self.kd_angular = self.get_parameter('ki_linear').value
        self.ki_linear = self.get_parameter('ki_angular').value

        # self.declare_parameter("Kd", 0.00)
        # Publisher for publishing area and deviation
        self.publisher_ = self.create_publisher(Twist,'/ball_data',10)
        self.publisher2_ = self.create_publisher(Twist,'/cmd_vel',10)
        # self.subscription = self.create_subscription(
        #   Int16MultiArray, 
        #   'drive_topic', 
        #   self.listener_callback, 
        #   10)
        # self.subscription # prevent unused variable warning

        # self.array = Int16MultiArray()
        # Timer to periodically run YOLOv5 inference
        # self.timer_ = self.create_timer(1.0, self.inference_callback)

        
        self.run()

    def pid_controller(self, error, previous_error, int_error, ki, kd, dt):
        control_action = self.kp_linear * error + ki * int_error + kd * ((error - previous_error) / dt)
        return control_action
        
        # Initialize YOLOv5 model
        # self.initialize_yolov5_model()    
    
    @smart_inference_mode()
    def run(
        self,
        weights=redblue_model_path,  # model path or triton URL
        source=0  ,  # file/dir/URL/glob/screen/0(webcam)
        data=ROOT / "data/coco128.yaml",  # dataset.yaml path
        imgsz=(640, 640),  # inference size (height, width)
        conf_thres=0.5,  # conidence threshold
        iou_thres=0.45,  # NMS IOU threshold
        max_det=1,  # maximum detections per image
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
        max_ball=0
        dev=0
        detections_ball = []
        detections_silo = []
        deviation_sum = 0
        ballfound = False
        silofound=False
        LinXb=0
        AngZpb=0
        silo_count = 0
        silo_positions = {}
        balls_positions={}
        ball_count=0
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
            twist_msg2= Twist()
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
                # if ballfound == False   :
                #     twist_msg.linear.z=2.0
                #     # No objects detected, set linear and angular velocities to zero
                #     twist_msg.linear.x = 0.0
                #     twist_msg.angular.z = 0.2
                #     print(f"ball is been seen : {ballfound}")
                #     self.publisher_.publish(twist_msg)  
                   
                if  silofound == False :
                  
                    twist_msg2.linear.z=1.0
                    twist_msg2.linear.x = 0.0
                    twist_msg2.angular.z = 0.5
                    print(f"silo is been seen : {silofound}")
                    self.publisher2_.publish(twist_msg2)   
                if len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

                    # Print results
                    for c in det[:, 5].unique():
                        n = (det[:, 5] == c).sum()  # detections per class
                        s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string
                    silo_count = 0
                    silo_positions = {}
                    leftmost_silo_position = None
                    leftmost_balls_position = None
                    leftmost_silo_deviation = 0
                    leftmost_silo_area=0
                    leftmost_ball_deviation = 0
                    leftmost_ball_area=0
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
                        area=-area
                        # # if label=="Red-ball":
                        # # detections_silo[0][1]=69
                        # if (-area)> -(max_ball):
                        #     max_ball=area
                        # if label == self.setupballname:
                        #     # if (-area)> -(max_ball):
                        #         # max_ball=area
                        #     print("parameter of name is working")
                        #     detections_ball.append((label, area, deviation, x1, y1, x2, y2))
                        #     center_x_ball = (xyxy[0] + xyxy[2]) / 2
                        #     center_y_ball = (xyxy[1] + xyxy[3]) / 2
        
                        # # Track the position of the detected silo
                        #     silo_positions[center_x_ball] = (center_x_ball, center_y_ball)
                            
                        #     ball_count += 1

                        # # sorted_silos = sorted(silo_positions.items(), reverse=True)
                        # sorted_balls = sorted(balls_positions.items())
                        # if sorted_balls:
                        #     num_balls = len(sorted_balls)
    
                        #     middle_index = num_balls // 2
                            
                           
                        #     leftmost_balls_position = sorted_balls[middle_index][1]
                        #     center_x_ball, center_y_ball= leftmost_balls_position
                        #     bdeviation = center_x_ball- im0.shape[1] // 2 
                        #     leftmost_ball_deviation = bdeviation
                        #     print(f"leftmost silo X: {center_x_ball} , leftmost silo Y: {center_y_ball}")
                        #     print(f"deviation of ball :  + {bdeviation}")
                        #     if label == "Red-ball":
                        #         x1, y1, x2, y2 = xyxy  # Extract bounding box coordinates
                        #         areaball = abs((x2 - x1) * (y2 - y1))  # Calculate area
                        #         leftmost_ball_area = -areaball
                        #         # area=max_ball
                            
                            
                        if label == self.setupballname:
                            print(f"object is a {self.setupballname}")
                            detections_silo.append((label, area, deviation, x1, y1, x2, y2))
                            center_x = (xyxy[0] + xyxy[2]) / 2
                            center_y = (xyxy[1] + xyxy[3]) / 2
        
                        # Track the position of the detected silo
                            silo_positions[center_x] = (center_x, center_y)
                            
                            silo_count += 1

                        # sorted_silos = sorted(silo_positions.items(), reverse=True)
                        sorted_silos = sorted(silo_positions.items())
                        for i in range(len(sorted_silos)):
                            print()
                        if sorted_silos:
                            num_silos = len(sorted_silos)
    
                            middle_index = num_silos // 2
                            
                           
                            leftmost_silo_position = sorted_silos[middle_index][1]
                            center_x, center_y = leftmost_silo_position
                            sdeviation = center_x - im0.shape[1] // 2 
                            leftmost_silo_deviation = sdeviation
                            print(f"leftmost silo X: {center_x} , leftmost silo Y: {center_y}")
                            print(f"deviation of silo :  + {sdeviation}")
                            if label == self.setupballname:
                                x1, y1, x2, y2 = xyxy  # Extract bounding box coordinates
                                area2 = abs((x2 - x1) * (y2 - y1))  # Calculate area
                                leftmost_silo_area = -area2
                        # # Assign numbers to each silo based on its position (from right to left)
                        # silo_numbers = {}
                        # for i, (center_x, _) in enumerate(sorted_silos, start=1):
                        #     silo_numbers[i] = silo_positions[center_x]
                        # print(silo_count)
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
                        # print(det)
                        
                        
                        LinXs=map(leftmost_silo_area,-90000,100, 0, 1)
                        AngZpbl= map(leftmost_silo_deviation, -250,self.setupdev, 0.5, 0)
                        # LinZsy=map(y,self.setupareasilo,0,1,0)
                        
                        print(leftmost_silo_area)  
                        print(leftmost_silo_deviation)   

                        # print("ball:")
                        # print(leftmost_ball_area)  
                        # print(leftmost_ball_deviation)  
                        # if ball_count==1:
                        #     ballfound=True
                        #     print(f"ball is been seen : {ballfound}")
                        #     # print("leftmost area:"+leftmost_silo_area)
                        # if ballfound ==True :
                        #             print(f"left most ball area :{leftmost_ball_area}")
                                    
                        #             # count_i=len(detections_silo)-1
                        #             if  label== self.setupballname and leftmost_ball_area < self.setupareaball:
                        #                 print("hi")
                        #                 twist_msg.linear.x = 0.0
                        #                 twist_msg.angular.z = 0.0
                        #                 twist_msg.linear.y = 0.0

                        #                 twist_msg.linear.z=2.0
                        #                 self.publisher_.publish(twist_msg)
                        #                 # self.destroy_node()
                        #                 # rclpy.shutdown()

                        #                 # twist_msg2.linear.x = 0.0
                        #                 # twist_msg2.angular.z = 0.0
                        #                 # twist_msg2.linear.y = 0.0
    
                        #                 # twist_msg2.linear.z=1.0
                        #                 # self.publisher2_.publish(twist_msg2)
                        #                 # twist_msg2.linear.x = 0.0
                        #                 # twist_msg2.angular.z = 0.0
                        #                 # twist_msg2.linear.y = 0.0

                        #                 # twist_msg2.linear.z=1.0
                        #                 # self.publisher2_.publish(twist_msg2)
                        #                 # twist_msg2.linear.x = 0.0
                        #                 # twist_msg2.angular.z = 0.0
                        #                 # twist_msg2.linear.y = 0.0

                        #                 # twist_msg2.linear.z=1.0
                        #                 # self.publisher2_.publish(twist_msg2)
                        #                 # twist_msg2.linear.x = 0.0
                        #                 # twist_msg2.angular.z = 0.0
                        #                 # twist_msg2.linear.y = 0.0

                        #                 # twist_msg2.linear.z=1.0
                        #                 # self.publisher2_.publish(twist_msg2)
                        #                 # twist_msg2.linear.x = 0.0
                        #                 # twist_msg2.angular.z = 0.0
                        #                 # twist_msg2.linear.z=1.0
                        #                 # twist_msg2.linear.y = 0.0

                        #                 # self.publisher2_.publish(twist_msg2)
                        #                 # twist_msg2.linear.x = 0.0
                        #                 # twist_msg2.angular.z = 0.0
                        #                 # twist_msg2.linear.z=1.0
                        #                 # twist_msg2.linear.y = 0.0

                        #                 # self.publisher2_.publish(twist_msg2)
                        #             elif   leftmost_ball_area>= self.setupareaball:
                        #                 print("positive area before = " + str(float(leftmost_ball_area)))
                        #                 # area=leftmost_silo_area
                        #                 area = (41000 + float(leftmost_ball_area))/1000
                        #                 # twist_msg2.linear.x = float(LinXs)
                        #                 # twist_msg2.angular.z = float(AngZpbl)
                        #                 # twist_msg.linear.z=2.0

                        #                 # twist_msg2.linear.x = float(LinXs)
                        #                 # twist_msg2.angular.z = float(AngZpbl)
                        #                 print(f"positive area =  {float(self.pid_controller(area,0,0,0,0,0.1))} \n")
                        #                 print("positive area of ball after = " + str(float(area)))
                        #                 # print("deviation ball= " + str(deviation/100))
                        #                 deviation = float(leftmost_ball_deviation)

                        #                 # twist_msg.linear.x = float(LinXb)
                        #                 # twist_msg.angular.z =- float(AngZpb)
                                        
                        #                 # self.publisher_.publish(twist_msg)
                        #                 twist_msg.linear.x = float(self.pid_controller(area,0,0,0,0,0.1))
                        #                 # twist_msg.linear.y =- float(self.pid_controller((deviation/40),0,0,0,0,0.1))
                        #                 twist_msg.angular.z = float(self.pid_controller((deviation/30),0,0,0,0,0.1))

                        #                 twist_msg.linear.x = max(min(twist_msg.linear.x, self.linear_x_max), self.linear_x_min)
                        #                 # twist_msg.linear.y = max(min(twist_msg.linear.y, self.linear_y_max), self.linear_y_min)
                        #                 twist_msg.angular.z = max(min(twist_msg.angular.z, self.angular_z_max), self.angular_z_min)
                        #                 twist_msg.linear.z=2.0
                        #                 # if(area == 50000):
                        #                 #     twist_msg2.linear.x = 0.0

                        #                 self.publisher_.publish(twist_msg)                                         
                        # # if len(detections_ball) > 0 :
                        # #     count_i=len(detections_ball)-1
                        # #     deviation=-deviation   
                        # #     AngZpb = map(detections_ball[count_i][2], -250,self.setupdev, 0.5, 0)
                        # #     LinXb=map(detections_ball[count_i][1], -50000,100, 0, 2)
                        # #     ballfound=True
                        # #     print(f"ball is been seen : {ballfound}")
                            
                        # #     if  detections_ball[count_i][0] == self.setupballname and detections_ball[count_i][1] >= self.setupareaball:
                                        
                        # #     # twist_msg.linear.x = float(LinXb)
                        # #     # twist_msg.angular.z = float(AngZpb)
                        # #     # self.publisher_.publish(twist_msg)
                        # #         # if deviation >=-200 and deviation <=200:
                        # #                 area = (41000 + float(detections_ball[count_i][1]))/1000
                        # #                 # twist_msg2.linear.x = float(LinXs)
                        # #                 # twist_msg2.angular.z = float(AngZpbl)
                        # #                 # twist_msg.linear.z=2.0

                        # #                 # twist_msg2.linear.x = float(LinXs)
                        # #                 # twist_msg2.angular.z = float(AngZpbl)
                        # #                 print(f"positive area =  {float(self.pid_controller(area,0,0,0,0,0.1))} \n")
                        # #                 print("positive area of ball after = " + str(float(area)))
                        # #                 # print("deviation ball= " + str(deviation/100))
                        # #                 deviation = float(detections_ball[count_i][2])

                        # #                 # twist_msg.linear.x = float(LinXb)
                        # #                 # twist_msg.angular.z =- float(AngZpb)
                                        
                        # #                 # self.publisher_.publish(twist_msg)
                        # #                 twist_msg.linear.x = float(self.pid_controller(area,0,0,0,0,0.1))
                        # #                 # twist_msg.linear.y =- float(self.pid_controller((deviation/40),0,0,0,0,0.1))
                        # #                 twist_msg.angular.z = float(self.pid_controller((deviation/30),0,0,0,0,0.1))

                        # #                 twist_msg.linear.x = max(min(twist_msg.linear.x, self.linear_x_max), self.linear_x_min)
                        # #                 # twist_msg.linear.y = max(min(twist_msg.linear.y, self.linear_y_max), self.linear_y_min)
                        # #                 twist_msg.angular.z = max(min(twist_msg.angular.z, self.angular_z_max), self.angular_z_min)
                        # #                 twist_msg.linear.z=2.0
                        # #                 # if(area == 50000):
                        # #                 #     twist_msg2.linear.x = 0.0

                        # #                 self.publisher_.publish(twist_msg)
                        # #         # # twist_msg.linear.y=float(LinZsy)
                        # #         # if (deviation >=-250 and deviation <=-200) or (deviation <=250 and deviation >=200)   :
                        # #         #     twist_msg.linear.x = float(LinXb/2)
                        # #         #     twist_msg.angular.z = float(AngZpb *2)      
                        # #         #     twist_msg.linear.z=2.0                                                                                                                                                                                                                                                 
                        # #         #     self.publisher_.publish(twist_msg)
                        # # # if len(detections_ball) > 0 and detections_ball[0][1] <= self.setupareaball and detections_ball[0][0] == "Red-ball":
                        # # #     # Robot holds the ball, move towards the silo
                        # # #     print("hi")

                        # # #     if label=="silo":
                        # # #         print("Moving towards silo while holding the ball")
                        # # #         # Calculate LinXs and AngZpb using your map function based on silo detection
                              
                        # # #         twist_msg.linear.x = float(LinXs)
                        # # #         twist_msg.angular.z = float(AngZpb)
                        # # # if  label=="":    
                        # # # twist_msg.angular.z = 1.0
                        # # # if label=="Red-ball" and detections_ball[0][1]>=self.setupareaball:
                        # # #     twist_msg.linear.x = float(LinXb)
                        # # #     twist_msg.angular.z = float(AngZpb)
                        
                        # #     if  detections_ball[count_i][0] == self.setupballname and detections_ball[count_i][1] <= self.setupareaball:
                        # #         twist_msg.linear.x = 0.0
                        # #         twist_msg.angular.z = 0.0
                        # #         twist_msg.linear.z=2.0
                        # #         self.publisher_.publish(twist_msg)
                        # #         # twist_msg.linear.x = 0.0
                        # #         # twist_msg.angular.z = 0.0
                        # #         # twist_msg.linear.z=2.0
                        # #         # self.publisher_.publish(twist_msg)
                        # #         # twist_msg.linear.x = 0.0
                        # #         # twist_msg.angular.z = 0.0
                        #         # twist_msg.linear.z=2.0
                        #         # self.publisher_.publish(twist_msg)
                        #         # twist_msg.linear.x = 0.0
                        #         # twist_msg.angular.z = 0.0
                        #         # twist_msg.linear.z=2.0
                        #         # self.publisher_.publish(twist_msg)
                        #         # twist_msg.linear.x = 0.0
                        #         # twist_msg.angular.z = 0.0
                        #         # twist_msg.linear.z=2.0
                        #         # self.publisher_.publish(twist_msg)
                        #         # twist_msg.linear.x = 0.0
                        #         # twist_msg.angular.z = 0.0
                        #         # twist_msg.linear.z=2.0
                        #         # self.publisher_.publish(twist_msg)
                        #         # twist_msg.linear.x = 0.0
                        #         # twist_msg.angular.z = 0.0
                        #         # twist_msg.linear.z=2.0
                        #         # self.publisher_.publish(twist_msg)
                        #     #     if  twist_msg.angular.z == 0.0   and twist_msg.linear.x == 0.0:
                        #     #         cou
                        #     # # # r = math.sqrt(area/3.14)
                        #         # count = count +1
                        #         # print(count)
                        #         # if count > 5:
                                # self.destroy_node()
                        
                        # print(len(detections_ball))
                        # print(len(detections_ball))
                        # print(len(detections_ball))
                        # if len(detections_ball)>0:
                        # if label=="silo" and not label=="Red-ball" :
                        #     twist_msg2.linear.z=1.0
                        #     twist_msg2.linear.x = 0.0
                        #     twist_msg2.angular.z = 0.0
                        #     print(f"silo is been seen : {silofound}")
                            
                            # self.publisher2_.publish(twist_msg2)  
                        if silofound ==True and label != self.setupballname:
                                twist_msg2.linear.z=0.0
                                twist_msg2.linear.x=0.0
                                twist_msg2.linear.y=0.0

                                print(f"silo is been seen out of frame : {silofound}")

                                # twist_msg2.angular.z = float(AngZpbl *2)
                                twist_msg2.angular.z = 0.5
                                self.publisher2_.publish(twist_msg2)   
                        if silo_count>=1:
                            silofound=True
                            print(f"silo is been seen : {silofound}")
                            # print("leftmost area:"+leftmost_silo_area)
                        if silofound ==True and leftmost_silo_position:
                                    print(f"left most silo area :{leftmost_silo_area}")
                                    
                                    # count_i=len(detections_silo)-1
                                    if  label== self.setupballname and leftmost_silo_area < self.setupareasilo:
                                        print("hi")
                                        twist_msg2.linear.x = 0.0
                                        twist_msg2.angular.z = 0.0
                                        twist_msg2.linear.y = 0.0

                                        twist_msg2.linear.z=1.0
                                        self.publisher2_.publish(twist_msg2)
                                        self.destroy_node()
                                        # rclpy.shutdown()

                                        # twist_msg2.linear.x = 0.0
                                        # twist_msg2.angular.z = 0.0
                                        # twist_msg2.linear.y = 0.0
    
                                        # twist_msg2.linear.z=1.0
                                        # self.publisher2_.publish(twist_msg2)
                                        # twist_msg2.linear.x = 0.0
                                        # twist_msg2.angular.z = 0.0
                                        # twist_msg2.linear.y = 0.0

                                        # twist_msg2.linear.z=1.0
                                        # self.publisher2_.publish(twist_msg2)
                                        # twist_msg2.linear.x = 0.0
                                        # twist_msg2.angular.z = 0.0
                                        # twist_msg2.linear.y = 0.0

                                        # twist_msg2.linear.z=1.0
                                        # self.publisher2_.publish(twist_msg2)
                                        # twist_msg2.linear.x = 0.0
                                        # twist_msg2.angular.z = 0.0
                                        # twist_msg2.linear.y = 0.0

                                        # twist_msg2.linear.z=1.0
                                        # self.publisher2_.publish(twist_msg2)
                                        # twist_msg2.linear.x = 0.0
                                        # twist_msg2.angular.z = 0.0
                                        # twist_msg2.linear.z=1.0
                                        # twist_msg2.linear.y = 0.0

                                        # self.publisher2_.publish(twist_msg2)
                                        # twist_msg2.linear.x = 0.0
                                        # twist_msg2.angular.z = 0.0
                                        # twist_msg2.linear.z=1.0
                                        # twist_msg2.linear.y = 0.0

                                        # self.publisher2_.publish(twist_msg2)
                                    elif   leftmost_silo_area> self.setupareasilo:
                                        print("positive negative before = " + str(float(leftmost_silo_area)))
                                        # area=leftmost_silo_area
                                        leftmost_silo_area = (-self.setupareasilo + float(leftmost_silo_area))/200
                                        # twist_msg2.linear.x = float(LinXs)
                                        # twist_msg2.angular.z = float(AngZpbl)
                                        # twist_msg2.linear.z=1.0

                                        # twist_msg2.linear.x = float(LinXs)
                                        # twist_msg2.angular.z = float(AngZpbl)
                                        # print(f"positive area =  {float(self.pid_controller(area,0,0,0,0,0.1))} \n")
                                        print("positive area after = " + str(float(leftmost_silo_area)))
                                        print("deviation 2 = " + str(leftmost_silo_deviation/100))
                                        leftmost_silo_deviation = float(leftmost_silo_deviation)

                                    # twist_msg.linear.x = float(LinXb)
                                    # twist_msg.angular.z = float(AngZpb)
                                    # self.publisher_.publish(twist_msg)
                                        twist_msg2.linear.x = float(self.pid_controller(leftmost_silo_area,0,0,0,0,0.1))
                                        twist_msg2.linear.y =- float(self.pid_controller((leftmost_silo_deviation/30),0,0,0,0,0.1))
                                        twist_msg2.angular.z = -float(self.pid_controller((leftmost_silo_deviation/30),0,0,0,0,0.1))

                                        twist_msg2.linear.x = max(min(twist_msg2.linear.x, self.linear_x_max), self.linear_x_min)
                                        twist_msg2.linear.y = max(min(twist_msg2.linear.y, self.linear_y_max), self.linear_y_min)
                                        twist_msg2.angular.z = max(min(twist_msg2.angular.z, self.angular_z_max), self.angular_z_min)
                                        twist_msg2.linear.z=1.0
                                        # if(area == 50000):
                                        #     twist_msg2.linear.x = 0.0

                                        self.publisher2_.publish(twist_msg2)
                            #     if  twist_msg.angular.z == 0.0   and twist_msg.linear.x == 0.0:
                            #         cou
                            # # # r = math.sqrt(area/3.14)
                                # count = count +1
                                # print(count)
                                # if count > 5:
                                # self.destroy_node()    
                                # self.get_logger().info("Enter in node of publisher")
                                
                                # Print class, area, and deviation
                        print(f"Class: {label}, Area: {area}, Deviation: {deviation} ")
                                # print(seen)
                                # print(len(detections_ball))
                            # print(detections_ball[count_i][0])    
                            # print(ballfound)

                           

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
                else:
                    
                    # if ballfound :
                    #     print(f"ball is been seen  out of frame : {ballfound}")
                    #     twist_msg.linear.z=2.0
                    #     # twist_msg.linear.x = float(LinXb)
                    #     twist_msg.angular.z = float(AngZpb *2)
                    #     self.publisher_.publish(twist_msg)
                    if silofound:
                        # if leftmost_ball_deviation > 
                        twist_msg2.linear.z=0.0
                        twist_msg2.linear.x=0.0
                        twist_msg2.linear.y=0.0

                        print(f"silo is been seen out of frame : {silofound}")

                        # twist_msg2.angular.z = float(AngZpbl *2)
                        twist_msg2.angular.z = 0.5
                        self.publisher2_.publish(twist_msg2)   

               
                # elif :
                #     # No objects detected, set linear and angular velocities to zero
                #     twist_msg.linear.z=1.0
                #     twist_msg.linear.x = 0.0
                #     twist_msg.angular.z = 0.5
                #     # print(ballfound)
                #     self.publisher_.publish(twist_msg)        
                # Stream results
                # elif :
                    # print(ballfound)
                    # twist_msg.linear.x = float(LinXb)
                     
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
