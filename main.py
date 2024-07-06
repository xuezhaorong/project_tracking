import cv2
from picamera2 import Picamera2
import time
import threading
from PIDCtrl import PIDCtrl
from ServoCtrl import ServoCtrl
import queue

servox, servoy = 17, 27
servo_anglex = 90
servo_angley = 20
dispW, dispH = 0, 0
SERVO_MAX_X_ANGLE = 150
SERVO_MIN_X_ANGLE = 30
SERVO_MAX_Y_ANGLE = 150
SERVO_MIN_Y_ANGLE = 20

Tracker_Type = {
    'BOOSTING': cv2.legacy.TrackerBoosting(),
    'MIL': cv2.legacy.TrackerMIL(),
    'KCF': cv2.legacy.TrackerKCF(),
    'TLD': cv2.legacy.TrackerTLD(),
    'MEDIANFLOW': cv2.legacy.TrackerMedianFlow(),
    'MOSSE': cv2.legacy.TrackerMOSSE(),
    'CSRT': cv2.legacy.TrackerCSRT()
}



def Servo_adjust(anglex, angley):
    servoCtrlX.Servo_SetAngle(anglex)
    servoCtrlY.Servo_SetAngle(angley)


if __name__ == "__main__":
    # 舵机
    servoCtrlX = ServoCtrl(servox, servo_anglex)
    servoCtrlY = ServoCtrl(servoy, servo_angley)

    picam2 = Picamera2()
    picam2_config = picam2.create_preview_configuration()
    picam2.configure(picam2_config)
    picam2.start()
    time.sleep(2.0)

    tracker = cv2.legacy.TrackerCSRT.create()
    # 读取第一帧
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    print("-----------------------------------------------", frame.shape[:2])
    dispW, dispH = frame.shape[:2]

    # PID 调参
    PIDCtrlX = PIDCtrl(0.1, 0.0, 0.019, dispW / 2)
    PIDCtrlY = PIDCtrl(0.05, 0.0, 0.03, dispH / 2)

    # roi
    bbox = cv2.selectROI('roi', frame, False, False)
    roi = frame[int(bbox[1]):int(bbox[1] + bbox[3]), int(bbox[0]):int(bbox[0] + bbox[2])]
    cv2.imshow('roi', roi)
    cv2.waitKey(0)
    # 初始化tracker
    result = tracker.init(frame, bbox)

    while True:
        # 获得图像
        frame = cv2.cvtColor(picam2.capture_array(), cv2.COLOR_BGRA2BGR)
        # 更新跟踪器
        result, bbox = tracker.update(frame)

        if result:
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            # 找到图像中心
            # bbox x y w h
            centerX = bbox[0] + int(bbox[2] / 2.0)
            centerY = bbox[1] + int(bbox[3] / 2.0)

            # PID计算
            angleX = PIDCtrlX.PID_Compute(centerX)
            angleY = PIDCtrlY.PID_Compute(centerY)

            servo_anglex += angleX
            # 限制角度
            servo_anglex = SERVO_MAX_X_ANGLE if servo_anglex > SERVO_MAX_X_ANGLE else SERVO_MIN_X_ANGLE if servo_anglex < SERVO_MIN_X_ANGLE else servo_anglex

            servo_angley += angleY
            # 限制角度
            servo_angley = SERVO_MAX_Y_ANGLE if servo_angley > SERVO_MAX_Y_ANGLE else SERVO_MIN_Y_ANGLE if servo_angley < SERVO_MIN_Y_ANGLE else servo_angley
            print(f"---------------------------------anglex:{servo_anglex},angley:{servo_angley}")

            servoCtrlX.Servo_SetAngle(int(servo_anglex))
            servoCtrlY.Servo_SetAngle(int(servo_angley))


            # 开启线程
            # t = threading.Thread(target=Servo_adjust, args=(servo_anglex, servo_angley))
            # t.start()

            cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)

        cv2.imshow('tracking', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):  # 若按下的字符为'q'时退出循环
            break

    ServoCtrl.Servo_clean()
    cv2.destroyAllWindows()

