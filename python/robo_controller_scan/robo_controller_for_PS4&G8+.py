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

class Robot:
    def __init__(self):
        self.path = [(0, 0)]  # 初期位置

    def move(self, dx, dy):
        current_x, current_y = self.path[-1]
        new_x = current_x + dx
        new_y = current_y + dy
        self.path.append((new_x, new_y))

    def return_to_start(self):
        current_x, current_y = self.path[-1]
        start_x, start_y = self.path[0]
        dx = start_x - current_x
        dy = start_y - current_y
        return dx, dy

    def print_path(self):
        print("ロボットの移動経路:")
        for i, (x, y) in enumerate(self.path):
            print(f"ステップ {i}: ({x}, {y})")

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
            ser.write(data.encode()) #ESP32 Lowerに送信
            print(f"Object {i}. {model.names[int(c)]}: X={center_x}, Y={center_y}, Confidence={conf:.2f}")

        img = annotator.result()
        cv2.imshow('YOLO V8 Detection', img)

if __name__ == "__main__":
    
    while True:  
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
        ser = serial.Serial("COM13", 2500000, timeout=0.1) #ESP32 Lower用
        ser1 = serial.Serial("COM5", 115200, timeout=0.1) #ESP32 Upper用
        time.sleep(2)
        
        robot = Robot() #関数の設定
        stop_flag1 = False  #停止フラグの設定
        stop_flag2 = False
        stop_flag3 = False
        stop_flag4 = False
        stop_flag5 = False
        stop_flag6 = False
        stop_flag7 = False
        
        triggerflag1 = False #トリガー用フラッグ
        triggerflag2 = False
        
        try:
            # 初動処理
            print("Press button to start...")
            while not stop_flag1 and not stop_flag2 and not stop_flag3 and not stop_flag4:
                _, img = cap.read()
                cv2.imshow('Press Button to Start', img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):  # qキー
                    stop_flag1 = True
                elif key == ord("r"):  # rキー
                    stop_flag2 = True
                elif key == ord("b"):
                    stop_flag3 = True  # bキー
                elif key == ord("c"):  
                    stop_flag4 = True  # cキー
            cv2.destroyAllWindows()
            
            # 赤フィールド処理
            while not stop_flag1 and not stop_flag3 and not stop_flag4 and not stop_flag5:
                
                while not stop_flag7:
                    print("red stage program...")
                    ser.write(f"redpro:red stage\n".encode()) 
                    ser1.write(f"redpro:red stage\n".encode())
                    stop_flag7 = True
                
                cv2.imshow('Dummy Window', np.zeros((1, 1), dtype=np.uint8))
                key = cv2.waitKey(1) & 0xFF
                if key == ord(' '):  # スペースキー
                    print("program start...")
                    ser.write(f"prostart:ALL READY\n".encode())
                    ser1.write(f"prostart:ALL READY\n".encode())
                    stop_flag5 = True
                elif key == ord("q"):  # qキー
                    stop_flag1 = True
                
            cv2.destroyAllWindows()
                
            while not stop_flag1 and not stop_flag3 and not stop_flag4 and not stop_flag6:
                cv2.imshow('Dummy Window', np.zeros((1, 1), dtype=np.uint8))
                ser.write(f"LYRX:{-25}:{0}\n".encode())  # 前に進む
                ser1.write(f"LYRX:{-25}:{0}\n".encode())
                start_time = time.time()
                while time.time() - start_time < 3:
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord(' '):
                        stop_flag6 = True
                        break
                    elif key == ord("q"):
                        stop_flag1 = True
                        break

                ser.write(f"R2A:{25}\n".encode())  # 右旋回
                ser1.write(f"R2A:{25}\n".encode())
                start_time = time.time()
                while time.time() - start_time < 1:
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord(' '):
                        stop_flag6 = True
                        break
                    elif key == ord("q"):
                        stop_flag1 = True
                        break

            cv2.destroyAllWindows()
            
            # 画像処理
            while not stop_flag1 and not stop_flag3 and not stop_flag4:
                robot.yolo_capture()
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):  # qキー
                    stop_flag1 = True
            
            # 青フィールド処理
            while not stop_flag1 and not stop_flag2 and not stop_flag4 and not stop_flag5:
                
                while not stop_flag7:
                    print("blue stage program...")
                    ser.write(f"bluepro:blue stage\n".encode())
                    ser1.write(f"bluepro:blue stage\n".encode())
                    stop_flag7 = True
                
                cv2.imshow('Dummy Window', np.zeros((1, 1), dtype=np.uint8))
                key = cv2.waitKey(1) & 0xFF
                if key == ord(' '):  # スペースキー
                    print("program start...")
                    ser.write(f"prostart:ALL READY\n".encode())
                    ser1.write(f"prostart:ALL READY\n".encode())
                    stop_flag5 = True
                elif key == ord("q"):  # qキー
                    stop_flag1 = True
                
            cv2.destroyAllWindows()
                
            while not stop_flag1 and not stop_flag2 and not stop_flag4 and not stop_flag6:
                cv2.imshow('Dummy Window', np.zeros((1, 1), dtype=np.uint8))
                ser.write(f"LYRX:{-25}:{0}\n".encode())  # 前に進む
                ser1.write(f"LYRX:{-25}:{0}\n".encode())
                start_time = time.time()
                while time.time() - start_time < 3:
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord(' '):
                        stop_flag6 = True
                        break
                    elif key == ord("q"):
                        stop_flag1 = True
                        break

                ser.write(f"R2A:{25}\n".encode())  # 右旋回
                ser1.write(f"R2A:{25}\n".encode())
                start_time = time.time()
                while time.time() - start_time < 1:
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord(' '):
                        stop_flag6 = True
                        break
                    elif key == ord("q"):
                        stop_flag1 = True
                        break

            cv2.destroyAllWindows()
            
            # 画像処理
            while not stop_flag1 and not stop_flag2 and not stop_flag4:
                robot.yolo_capture()
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):  # qキー
                    stop_flag1 = True
            
            # 手動フィールド処理
            while not stop_flag1 and not stop_flag2 and not stop_flag3 and not stop_flag5:
                
                while not stop_flag7:
                    print("controller stage program...")
                    ser.write(f"conpro:controller\n".encode())
                    ser1.write(f"conpro:controller\n".encode())
                    stop_flag7 = True
                
                cv2.imshow('Dummy Window', np.zeros((1, 1), dtype=np.uint8))
                key = cv2.waitKey(1) & 0xFF
                if key == ord(' '):  # スペースキー
                    print("program start...")
                    ser.write(f"prostart:ALL READY\n".encode())
                    ser1.write(f"prostart:ALL READY\n".encode())
                    stop_flag5 = True
                elif key == ord("q"):  # qキー
                    stop_flag1 = True
                
            cv2.destroyAllWindows()
                
            while not stop_flag1 and not stop_flag2 and not stop_flag3 and not stop_flag6:
                    events = pygame.event.get()
                    for event in events:
                        if event.type == pygame.JOYBUTTONDOWN:  # ボタンが押された場合
                            if event.button == 2:
                                print("□ボタンが押されました")  #G8→Yボタン
                                ser.write(f"SB:{1}\n".encode()) 
                                ser1.write(f"SB:{1}\n".encode()) 
                                time.sleep(0.05)
                            if event.button == 0:
                                print("バツボタンが押されました") #G8→Bボタン
                                ser.write(f"XB:{1}\n".encode())
                                ser1.write(f"XB:{1}\n".encode())
                                time.sleep(0.05)
                            if event.button == 1:
                                print("マルボタンが押されました") #G8→Aボタン
                                ser.write(f"CB:{1}\n".encode())
                                ser1.write(f"CB:{1}\n".encode())
                                time.sleep(0.05)
                            if event.button == 3:
                                print("△ボタンが押されました") #G8→Xボタン
                                ser.write(f"TB:{1}\n".encode())
                                ser1.write(f"TB:{1}\n".encode())
                                time.sleep(0.05)
                            if event.button == 4:
                                print("SHEARボタンが押されました")
                                ser.write(f"MB:{1}\n".encode())
                                ser1.write(f"MB:{1}\n".encode())
                                time.sleep(0.05)
                            if event.button == 5:
                                print("PSボタンが押されました")
                                ser.write(f"HB:{1}\n".encode())
                                ser1.write(f"HB:{1}\n".encode())
                                time.sleep(0.05)
                            if event.button == 6:
                                print("OPTIONボタンが押されました")
                                ser.write(f"PB:{1}\n".encode())
                                ser1.write(f"PB:{1}\n".encode())
                                time.sleep(0.05)
                            if event.button == 7:
                                print("左スティックが押されました")
                                ser.write(f"SLB:{1}\n".encode())
                                ser1.write(f"SLB:{1}\n".encode())
                                time.sleep(0.05)
                            if event.button == 8:
                                print("右スティックが押されました")
                                ser.write(f"SRB:{1}\n".encode())
                                ser1.write(f"SRB:{1}\n".encode())
                                time.sleep(0.05)
                            if event.button == 9:
                                print("L1が押されました")
                                ser.write(f"L1B:{1}\n".encode())
                                ser1.write(f"L1B:{1}\n".encode())
                                time.sleep(0.05)
                            if event.button == 10:
                                print("R1が押されました")
                                ser.write(f"R1B:{1}\n".encode())
                                ser1.write(f"R1B:{1}\n".encode())
                                time.sleep(0.05)
                            if event.button == 11:
                                print("上ボタンが押されました")
                                ser.write(f"UB:{1}\n".encode())
                                ser1.write(f"UB:{1}\n".encode())
                                time.sleep(0.05)
                            if event.button == 12:
                                print("下ボタンが押されました")
                                ser.write(f"DB:{1}\n".encode())
                                ser1.write(f"DB:{1}\n".encode())
                                time.sleep(0.05)
                            if event.button == 13:
                                print("左ボタンが押されました")
                                ser.write(f"LB:{1}\n".encode())
                                ser1.write(f"LB:{1}\n".encode())
                                time.sleep(0.05)
                            if event.button == 14:
                                print("右ボタンが押されました")
                                ser.write(f"RB:{1}\n".encode())
                                ser1.write(f"RB:{1}\n".encode())
                                time.sleep(0.05)
                            if event.button == 15:
                                print("タッチパッドが押されました")
                                ser.write(f"TAB:{1}\n".encode())
                                ser1.write(f"TAB:{1}\n".encode())
                                time.sleep(0.05)
                        if event.type == pygame.JOYBUTTONUP: #ボタンが離された場合
                            if event.button == 2:
                                print("□ボタンが離されました")
                                ser.write(f"SB:{0}\n".encode())
                                ser1.write(f"SB:{0}\n".encode())
                                time.sleep(0.05)
                            if  event.button == 0:
                                print("バツボタンが離されました")
                                ser.write(f"XB:{0}\n".encode())
                                ser1.write(f"XB:{0}\n".encode())
                                time.sleep(0.05)
                            if  event.button == 1:
                                print("マルボタンが離されました")
                                ser.write(f"CB:{0}\n".encode())
                                ser1.write(f"CB:{0}\n".encode())
                                time.sleep(0.05)
                            if event.button == 3:
                                print("△ボタンが離されました")
                                ser.write(f"TB:{0}\n".encode())
                                ser1.write(f"TB:{0}\n".encode())
                                time.sleep(0.05)
                            if event.button == 4:
                                print("SHEARボタンが離されました")
                                ser.write(f"MB:{0}\n".encode())
                                ser1.write(f"MB:{0}\n".encode())
                                time.sleep(0.05)
                            if event.button == 5:
                                print("PSボタンが離されました")
                                ser.write(f"HB:{0}\n".encode())
                                ser1.write(f"HB:{0}\n".encode())
                                time.sleep(0.05)
                            if event.button == 6:
                                print("OPTIONボタンが離されました")
                                ser.write(f"PB:{0}\n".encode())
                                ser1.write(f"PB:{0}\n".encode())
                                time.sleep(0.05)
                            if event.button == 7:
                                print("左スティックが離されました")
                                ser.write(f"SLB:{0}\n".encode())
                                ser1.write(f"SLB:{0}\n".encode())
                                time.sleep(0.05)
                            if event.button == 8:
                                print("右スティックが離されました")
                                ser.write(f"SRB:{0}\n".encode())
                                ser1.write(f"SRB:{0}\n".encode())
                                time.sleep(0.05)
                            if event.button == 9:
                                print("L1が離されました")
                                ser.write(f"L1B:{0}\n".encode())
                                ser1.write(f"L1B:{0}\n".encode())
                                time.sleep(0.05)
                            if event.button == 10:
                                print("R1が離されました")
                                ser.write(f"R1B:{0}\n".encode())
                                ser1.write(f"R1B:{0}\n".encode())
                                time.sleep(0.05)
                            if event.button == 11:
                                print("上ボタンが離されました")
                                ser.write(f"UB:{0}\n".encode())
                                ser1.write(f"UB:{0}\n".encode())
                                time.sleep(0.05)
                            if event.button == 12:
                                print("下ボタンが離されました")
                                ser.write(f"DB:{0}\n".encode())
                                ser1.write(f"DB:{0}\n".encode())
                                time.sleep(0.05)
                            if event.button == 13:
                                print("左ボタンが離されました")
                                ser.write(f"LB:{0}\n".encode())
                                ser1.write(f"LB:{0}\n".encode())
                                time.sleep(0.05)
                            if event.button == 14:
                                print("右ボタンが離されました")
                                ser.write(f"RB:{0}\n".encode())
                                ser1.write(f"RB:{0}\n".encode())
                                time.sleep(0.05)
                            if event.button == 15:
                                print("タッチパッドが離されました")
                                ser.write(f"TAB:{0}\n".encode())
                                ser1.write(f"TAB:{0}\n".encode())
                                time.sleep(0.05)
                        if event.type == pygame.JOYAXISMOTION: #ジョイスティック操作
                            if abs(j.get_axis(3)) >0:
                                rs_y = math.floor(j.get_axis(3) * 255)
                                ser.write(f"RSY:{rs_y}\n".encode())
                                ser1.write(f"RSY:{rs_y}\n".encode())
                                print(f"RSY:{rs_y}")
                            if abs(j.get_axis(1)) >0 or abs(j.get_axis(0)) >0 or abs(j.get_axis(2)) > 0:
                                ls_y = (j.get_axis(1) * 12)
                                ls_x = (j.get_axis(0) * 12)
                                rs_x = (j.get_axis(2) * 80)
                                ser.write(f"LYLXRX:{ls_y:.2f}:{ls_x:.2f}:{rs_x:.2f}\n".encode())
                                ser1.write(f"LYLXRX:{ls_y:.2f}:{ls_x:.2f}:{rs_x:.2f}\n".encode())
                                print(f"LYLXRX:{(ls_y):.2f}:{(ls_x):.2f}:{(rs_x):.2f}")
                            if j.get_axis(5) > 0 and not triggerflag1:
                                r2_a = math.floor((j.get_axis(5) + 1)*30/2)
                                ser.write(f"R2A:{1}\n".encode())
                                ser1.write(f"R2A:{1}\n".encode())
                                triggerflag1 = True
                                print(f"R2A:{r2_a}")
                            if j.get_axis(5) <= 0 and triggerflag1:
                                triggerflag1 = False
                            if j.get_axis(4) > 0 and not triggerflag2:
                                l2_a = math.floor((j.get_axis(4) + 1)*30/2)
                                ser.write(f"L2A:{1}\n".encode())
                                ser1.write(f"L2A:{1}\n".encode())
                                triggerflag2 = True
                                print(f"L2A:{l2_a}")
                            if j.get_axis(4) <= 0 and triggerflag2:
                                triggerflag2 = False
                        if event.type == pygame.KEYDOWN: #キーボード押された操作
                            if event.key == pygame.K_q:
                                print("qキーが押されました")
                                stop_flag1 = True
                                break                         
                            if event.key == pygame.K_h:
                                print("hキーが押されました")
                                ser.write(f"KH:{1}\n".encode())
                                ser1.write(f"KH:{1}\n".encode())                             
                            if event.key == pygame.K_l:
                                print("lキーが押されました")
                                ser.write(f"KL:{1}\n".encode())
                                ser1.write(f"KL:{1}\n".encode())
                            if event.key == pygame.K_d:
                                print("dキーが押されました")
                                ser.write(f"KD:{1}\n".encode())
                                ser1.write(f"KD:{1}\n".encode())
                            if event.key == pygame.K_s:
                                print("sキーが押されました")
                                ser.write(f"KS:{1}\n".encode())
                                ser1.write(f"KS:{1}\n".encode())
                            if event.key == pygame.K_t:
                                print("tキーが押されました")
                                ser.write(f"KT:{1}\n".encode())
                                ser1.write(f"KT:{1}\n".encode())
                        if event.type == pygame.KEYUP: #キーボード離された操作
                            if event.key == pygame.K_h:
                                print("hキーが離されました")
                                ser.write(f"KH:{0}\n".encode())
                                ser1.write(f"KH:{0}\n".encode())                             
                            if event.key == pygame.K_l:
                                print("lキーが離されました")
                                ser.write(f"KL:{0}\n".encode())
                                ser1.write(f"KL:{0}\n".encode())
                            if event.key == pygame.K_d:
                                print("dキーが離されました")
                                ser.write(f"KD:{0}\n".encode())
                                ser1.write(f"KD:{0}\n".encode())
                            if event.key == pygame.K_s:
                                print("sキーが離されました")
                                ser.write(f"KS:{0}\n".encode())
                                ser1.write(f"KS:{0}\n".encode())
                            if event.key == pygame.K_t:
                                print("tキーが離されました")
                                ser.write(f"KT:{0}\n".encode())
                                ser1.write(f"KT:{0}\n".encode())
                    time.sleep(0.05)

        except Exception as e:
            print(f"エラーが発生しました: {e}")

        finally:
            cap.release()
            cv2.destroyAllWindows()
            ser.close()
        
        if stop_flag1:
            print("Restarting...")
            continue  # メインループの最初に戻る
        else:
            break  # 正常終了の場合はループを抜ける
        
