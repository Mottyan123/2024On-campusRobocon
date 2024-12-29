import flet as ft
import random
import test_scan
import cv2

cap = cv2.VideoCapture(0)#キャプチャーの読み込み

class Robot:
    def __init__(self):
        self.x = 0
        self.y = 0

    def update_position(self):
        # ここで実際のロボットの位置を更新するロジックを追加します
        self.x += random.randint(-1, 1)
        self.y += random.randint(-1, 1)

def main(page: ft.Page):
    robot = Robot()

    position_label = ft.Text(value="Position: (0, 0)", size=24)

    def update_display(e):
        robot.update_position()
        position_label.value = f"Position: ({robot.x}, {robot.y})"
        page.update()

    update_button = ft.ElevatedButton(text="Update Position", on_click=update_display)
    
    def on_button_click(e):
        button.disabled = True
        page.update()
        test_scan.test_scan()

    button = ft.ElevatedButton(text="Yolo_v8 start", on_click=on_button_click)
    
    def on_button_click1(e):
        test_scan.clickbutton()
    
    button1 = ft.ElevatedButton(text="quit", on_click=on_button_click1)

    page.add(position_label, update_button,button,button1)

ft.app(target=main)

