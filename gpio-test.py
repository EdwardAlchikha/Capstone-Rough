import RPi.GPIO as GPIO
from picamera import PiCamera
import time
import signal
import sys
import threading


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
        camera.exposure_mode = 'antishake'
        camera.start_recording('/home/pi/test.h264')
        while not self.stopped():
            time.sleep(1)

        camera.stop_recording()


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
