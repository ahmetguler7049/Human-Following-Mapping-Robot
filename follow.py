import Jetson.GPIO as GPIO
import cv2
import time
import numpy as np
import cv2

import imutils

defaultSpeed = 30
windowCenter = 320
centerBuffer = 10
pwmBound = float(50)
cameraBound = float(320)
kp = pwmBound / cameraBound
leftBound = int(windowCenter + centerBuffer)
rightBound = int(windowCenter - centerBuffer)
error = 0
targetPixel = 0

# body_cascade = cv2.CascadeClassifier("fullbody.xml")


def main():
    # GPIO
    # import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    # Pin definitions
    rightFwd = 37
    rightRev = 38
    leftFwd = 35
    leftRev = 36
    #
    GPIO.setup(33, GPIO.OUT)  # sol teker hız pini
    GPIO.setup(32, GPIO.OUT)
    # GPIO initialization
    GPIO.setup(leftFwd, GPIO.OUT)
    GPIO.setup(leftRev, GPIO.OUT)
    GPIO.setup(rightFwd, GPIO.OUT)
    GPIO.setup(rightRev, GPIO.OUT)

    # Disable movement at startup
    GPIO.output(leftFwd, GPIO.LOW)
    GPIO.output(leftRev, GPIO.LOW)
    GPIO.output(rightFwd, GPIO.LOW)
    GPIO.output(rightRev, GPIO.LOW)

    # PWM Initialization

    sol = GPIO.PWM(33, 100)  # frekans aralığı belirlendi
    sag = GPIO.PWM(32, 100)

    sol.start(0)  # frekans üretimi aktif edildi
    sag.start(0)


    def updatePwm(rightPwm, leftPwm):
        sol.ChangeDutyCycle(leftPwm)
        sag.ChangeDutyCycle(rightPwm)


    def pwmStop():
        sag.ChangeDutyCycle(0)
        sol.ChangeDutyCycle(0)
        sag.stop()
        sol.stop()


    # sensor part

    # TRIG = 17
    # ECHO = 27

    # GPIO.setup(TRIG, GPIO.OUT)
    # GPIO.setup(ECHO, GPIO.IN)

    # GPIO.output(TRIG, False)


    # def sonar():
    #     start = 0
    #     stop = 0
    #     # SetPins
    #     GPIO.setup(TRIG, GPIO.OUT)
    #     GPIO.setup(ECHO, GPIO.IN)
    #
    #     GPIO.output(TRIG, True)
    #
    #     time.sleep(0.00001)  # 10uf call
    #
    #     GPIO.output(TRIG, False)
    #
    #     begin = time.time()
    #
    #     while GPIO.input(ECHO) == 0 and time.time() < begin + 0.05:
    #         start = time.time()
    #
    #     while GPIO.input(ECHO) == 1 and time.time() < begin + 0.1:
    #         stop = time.time()
    #
    #     elapsed = stop - start
    #
    #     distance = elapsed * 17500
    #
    #     return distance


    # Camera setup
    camera = cv2.VideoCapture(0)

    time.sleep(0.1)

    lower_yellow = np.array([24, 100, 90])
    upper_yellow = np.array([44, 255, 255])
    kernel = np.ones((3, 3), np.uint8)  # for mask
    font = cv2.FONT_HERSHEY_SIMPLEX
    # dis=sonar()
    try:
        while True:
            ret, frame = camera.read()

            frame = cv2.flip(frame, 1)  # flip 180 degree

            frame = cv2.GaussianBlur(frame, (5, 5), 0)  # smoothing

            # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # bodies = body_cascade.detectMultiScale(gray, 1.1, 1)

            # for (x, y, w, h) in bodies:
            #     cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            cnts = imutils.grab_contours(cnts)  # denoising

            frame_x, frame_y, _ = frame.shape

            # if camera find the ball
            if len(cnts) > 0:
                c = max(cnts, key=cv2.contourArea)  # maximum contour

                (x, y, w, h) = cv2.boundingRect(c)
                # x and y starting point, w and h height and width ratio

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)  # min rectangular drawing

                frame = cv2.putText(frame, 'Person', (x + w - 20, y + h + 30), font, 1, (255, 0, 255), 1, cv2.LINE_AA)

                min_area = w * h  # rectangular area

                x_point = x + (w / 2)  # obje center

                y_point = y + (h / 2)

                dis = 21  # distance

                targetPixel = x_point

                # Proportional controller
                print("min_area", min_area)

                #         print("w: ",int(w/2))
                #         print("h: ",h/2)

                if min_area > 15000:
                    print("Dis:", dis)

                    if (targetPixel < rightBound) or (targetPixel > leftBound):
                        error = windowCenter - targetPixel
                        pwmOut = abs(error * kp)
                        print(targetPixel)
                        turnPwm = pwmOut + defaultSpeed
                        if targetPixel < rightBound:
                            print("right side")
                            if 0 < targetPixel < 110:
                                print("targetPixel")
                                updatePwm(defaultSpeed, 20)
                                GPIO.output(leftFwd, 1)
                                GPIO.output(leftRev, 0)
                                GPIO.output(rightFwd, 0)
                                GPIO.output(rightRev, 1)

                            else:
                                updatePwm(defaultSpeed, turnPwm)
                                GPIO.output(leftFwd, 1)
                                GPIO.output(leftRev, 0)
                                GPIO.output(rightFwd, 1)
                                GPIO.output(rightRev, 0)
                        elif targetPixel > leftBound:
                            print("left side")
                            if 530 < targetPixel < 640:
                                print("targetPixel")
                                updatePwm(20, defaultSpeed)
                                GPIO.output(leftFwd, 0)
                                GPIO.output(leftRev, 1)
                                GPIO.output(rightFwd, 1)
                                GPIO.output(rightRev, 0)
                            else:
                                updatePwm(turnPwm, defaultSpeed)
                                GPIO.output(leftFwd, 1)
                                GPIO.output(leftRev, 0)
                                GPIO.output(rightFwd, 1)
                                GPIO.output(rightRev, 0)
                    else:
                        print("middle")
                        updatePwm(defaultSpeed, defaultSpeed)
                        GPIO.output(leftFwd, 1)
                        GPIO.output(leftRev, 0)
                        GPIO.output(rightFwd, 1)
                        GPIO.output(rightRev, 0)

                        print('go ahead', 'min area: =  ', min_area)

                elif min_area < 15000:

                    GPIO.output(leftFwd, 0)
                    GPIO.output(leftRev, 0)
                    GPIO.output(rightFwd, 0)
                    GPIO.output(rightRev, 0)
                    print('target is so close', dis)
            else:
                GPIO.output(leftFwd, 0)
                GPIO.output(leftRev, 0)
                GPIO.output(rightFwd, 0)
                GPIO.output(rightRev, 0)
                print('there is no target, searching')

            cv2.imshow("Frame", frame)

            # rawCapture.truncate(0)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                GPIO.output(leftFwd, 0)
                GPIO.output(leftRev, 0)
                GPIO.output(rightFwd, 0)
                GPIO.output(rightRev, 0)

                GPIO.cleanup()

                break

    finally:
        camera.release()
        cv2.destroyAllWindows()
        pwmStop()
        GPIO.cleanup()


if __name__ == '__main__':
    main()
