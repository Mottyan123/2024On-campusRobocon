from ultralytics import YOLO

#学習したyoloの読み込み(train_xxのバージョンを確認すること！！！)
model = YOLO("yolov8s.yaml")

results = model.train(data='C:/Users/hkazu/OneDrive/ドキュメント/ロボプロ/python/lobo_scan_model_s/datasets/dust_main/dust.yaml', epochs=500, patience=0, batch=32, device='cpu')

