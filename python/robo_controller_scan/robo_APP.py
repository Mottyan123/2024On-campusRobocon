import pygame
import time
import math
import numpy as np
import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0" #初期設定
import cv2
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import serial
import flet as ft

class Robot:

    def yolo_capture(self):
        _, img = cap.read()
        results = model.predict(img)
        objects = []
        for r in results:
            annotator = Annotator(img)
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0]  
                c = box.cls
                conf = box.conf[0] 
                if int(c) == 0 and conf >= 0.6:  
                    label = f"{model.names[int(c)]} {conf:.2f}" 
                    center_x = math.floor((b[0] + b[2]) / 2)
                    center_y = math.floor((b[1] + b[3]) / 2)
                    distance = math.sqrt((center_x - REFERENCE_POINT[0])**2 + (center_y - REFERENCE_POINT[1])**2)
                    objects.append((distance, (b, c, conf, label, center_x, center_y)))

        # 距離でソート
        objects.sort(key=lambda x: x[0])

        # 番号を割り当て
        for i, (distance, (b, c, conf, label, center_x, center_y)) in enumerate(objects, start=1):
            label_with_number = f"{i}. {label}"
            annotator.box_label(b, label_with_number)
            data = f"{i}.{model.names[int(c)]}:{center_x}:{center_y}\n"
            ser.write(data.encode())
            print(f"Object {i}. {model.names[int(c)]}: X={center_x}, Y={center_y}, Confidence={conf:.2f}")

        img = annotator.result()
        cv2.imshow('YOLO V8 Detection', img)

cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)
model = YOLO('../colab/02/last.pt')
REFERENCE_POINT = (320, 240)  # 画像の中心 (640x480の場合)
# Pygameの初期化
pygame.init()
pygame.joystick.init()
j = pygame.joystick.Joystick(0)
j.init()
screen = pygame.display.set_mode((1, 1))
pygame.display.set_caption("Keyboard Input Example")
print("pygame初期化完了")
print("コントローラーの準備が完了しました")
ser = serial.Serial("COM9", 115200, timeout=0.1)
#ser1 = serial.Serial("COM5", 115200, timeout=0.1)
time.sleep(2)

robot = Robot() #関数の設定