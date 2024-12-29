import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import cv2


# カメラキャプチャオブジェクトの作成
cap = cv2.VideoCapture(0)  # 0はデフォルトカメラを指します

# カメラの解像度を設定
#cap.set(3, 640)  # 横幅を640ピクセルに設定
#cap.set(4, 480)  # 縦幅を480ピクセルに設定

# キャプチャの例
while True:
    ret, frame = cap.read()
    if not ret:
        break
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# キャプチャオブジェクトとウィンドウを解放
cap.release()
cv2.destroyAllWindows()
