#!/usr/bin/env python3
import cv2


def main():
    for i in range(50):
        cap = cv2.VideoCapture(i)
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        if cap.isOpened():
            print("port:", "/dev/video"+str(i))
            while True:
                ret, frame = cap.read()
                cv2.imshow("/dev/video"+str(i), frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    # ros_operator = RosOperator()
    # if ros_operator.init_camera():
    #     print("camera opened")
    #     ros_operator.run()
    # else:
    #     print("camera error")

if __name__ == '__main__':
    main()

