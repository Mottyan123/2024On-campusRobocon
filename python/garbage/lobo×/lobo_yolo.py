from ultralytics import YOLO

model = YOLO('yolov8n.pt')#yolo読み込み

results = model.train(data='dust.yaml', epochs=100)#yamlファイルの学習

