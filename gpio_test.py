import RPi.GPIO as GPIO
from picamera import PiCamera
from picamera.array import PiRGBArray
import numpy as np
import importlib
import cv2
import time
import signal
import sys
import threading
from img_test import *


def drive(speed):
    if abs(speed) < 0.3:
        forwardPwm.ChangeFrequency(pwmFrequencySlow)
        backwardPwm.ChangeFrequency(pwmFrequencySlow)
    else:
        forwardPwm.ChangeFrequency(pwmFrequency)
        backwardPwm.ChangeFrequency(pwmFrequency)

    speed_adj = speed * 100
    forwardPwm.ChangeDutyCycle(max(min(speed_adj, 100), 0))
    backwardPwm.ChangeDutyCycle(max(min(-speed_adj, 100), 0))


def turn(angle):
    right = GPIO.LOW
    left = GPIO.LOW

    if angle > 0:
        right = GPIO.HIGH
    elif angle < 0:
        left = GPIO.HIGH

    GPIO.output(turnRightPin, right)
    GPIO.output(turnLeftPin, left)


def sigint_handler(sig, frame):
    print('\nCtrl+C pressed.\nStopping background threads.')
    navThread.stop()
    camThread.stop()

    forwardPwm.stop()
    backwardPwm.stop()

    navThread.join()
    camThread.join()

    GPIO.output(motorEnablePin, GPIO.LOW)
    turn(0)
    GPIO.output(turnEnablePin, GPIO.LOW)
    # GPIO.cleanup()
    print('Closing program.')
    sys.exit(0)


class StoppableThread(threading.Thread):
    def __init__(self):
        super(StoppableThread, self).__init__()
        self.stopEvent = threading.Event()

    def stop(self):
        self.stopEvent.set()

    def stopped(self):
        return self.stopEvent.is_set()


class CameraThread(StoppableThread):
    def __init__(self):
        super(CameraThread, self).__init__()

    def run(self):
        camera = PiCamera()
        camera.resolution = (640, 480)
        camera.framerate = 20
        raw_capture = PiRGBArray(camera, size=(640, 480))

        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        video_writer = cv2.VideoWriter('/home/pi/output.avi', fourcc, 10.0, (640, 480))

        time.sleep(0.2)  # give camera time to warm up?

        frame_counter = 0
        start_time = time.time()

        for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
            if self.stopped():
                break

            frame_counter += 1
            image = frame.array
            c = grey_and_blur(image)

            video_writer.write(cv2.cvtColor(c, cv2.COLOR_GRAY2BGR))
            raw_capture.truncate(0)

        end_time = time.time()
        elapsed_time = end_time - start_time

        print('releasing! ' + str(elapsed_time) + ' , ' + str(frame_counter))
        video_writer.release()
        cv2.destroyAllWindows()


class NavigationThread(StoppableThread):
    def __init__(self):
        super(NavigationThread, self).__init__()

    def run(self):
        while not self.stopped():
            drive(0.85)
            turn(-1)
            time.sleep(1)

            drive(-0.1)
            turn(1)
            time.sleep(1)


if __name__ == '__main__':
    GPIO.setmode(GPIO.BOARD)  # see https://pinout.xyz/, using in-order scheme

    motorForwardPin = 5
    motorBackwardPin = 7
    motorEnablePin = 11
    turnEnablePin = 12
    turnLeftPin = 10
    turnRightPin = 8

    pwmFrequency = 60  # Hz
    pwmFrequencySlow = 15  # Hz

    GPIO.setup(motorForwardPin, GPIO.OUT)
    GPIO.setup(motorBackwardPin, GPIO.OUT)
    GPIO.setup(motorEnablePin, GPIO.OUT)
    GPIO.setup(turnEnablePin, GPIO.OUT)
    GPIO.setup(turnLeftPin, GPIO.OUT)
    GPIO.setup(turnRightPin, GPIO.OUT)

    forwardPwm = GPIO.PWM(motorForwardPin, pwmFrequency)
    backwardPwm = GPIO.PWM(motorBackwardPin, pwmFrequency)
    GPIO.output(motorEnablePin, GPIO.HIGH)
    GPIO.output(turnEnablePin, GPIO.HIGH)
    forwardPwm.start(0)
    backwardPwm.start(0)

    signal.signal(signal.SIGINT, sigint_handler)

    navThread = NavigationThread()
    navThread.start()
    camThread = CameraThread()
    camThread.start()

    while True:
        time.sleep(1)
