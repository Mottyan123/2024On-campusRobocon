from ultralytics import YOLO

model = YOLO('yolov8n.pt')#yolo読み込み

results = model('./WIN_20240521_19_51_36_Pro.jpg')#画像認識を開始

from PIL import Image

for r in results:
    im_array = r.plot()
    im = Image.fromarray(im_array[..., ::-1])
    im.show()
    im.save('results_lobo.jpg')

