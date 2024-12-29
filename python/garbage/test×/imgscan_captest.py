import cv2

cap = cv2.VideoCapture(1)

while True:
    check, frame = cap.read()
    cv2.imshow("cap",frame)
    key = cv2.waitKey(1)  

    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows