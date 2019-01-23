import RPi.GPIO as GPIO
from picamera import PiCamera
import time
import signal
import sys
import threading


def drive(speed):
    if speed < 0:
        forwardPwm.ChangeDutyCycle(0)
        backwardPwm.ChangeDutyCycle(min(-speed * 100, 100))
    elif speed > 0:
        forwardPwm.ChangeDutyCycle(min(speed * 100, 100))
        backwardPwm.ChangeDutyCycle(0)
    else:
        forwardPwm.ChangeDutyCycle(0)
        backwardPwm.ChangeDutyCycle(0)


def sigint_handler(sig, frame):
    print('\nCtrl+C pressed.\nStopping background threads.')
    navThread.stop()
    camThread.stop()

    forwardPwm.stop()
    backwardPwm.stop()

    navThread.join()
    camThread.join()

    GPIO.output(motorEnablePin, GPIO.LOW)
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
            print('navThread: ' + str(time.time()))
            time.sleep(1)
            drive(-0.25)
            print('navThread: ' + str(time.time()))
            time.sleep(1)


if __name__ == '__main__':
    GPIO.setmode(GPIO.BOARD)  # see https://pinout.xyz/, using in-order scheme

    motorForwardPin = 5
    motorBackwardPin = 7
    motorEnablePin = 11

    pwmFrequency = 50  # Hz

    GPIO.setup(motorForwardPin, GPIO.OUT)
    GPIO.setup(motorBackwardPin, GPIO.OUT)
    GPIO.setup(motorEnablePin, GPIO.OUT)

    forwardPwm = GPIO.PWM(motorForwardPin, pwmFrequency)
    backwardPwm = GPIO.PWM(motorBackwardPin, pwmFrequency)

    signal.signal(signal.SIGINT, sigint_handler)

    GPIO.output(motorEnablePin, GPIO.HIGH)
    forwardPwm.start(0)
    backwardPwm.start(0)

    testCounter = 0
    lastTime = time.time()

    navThread = NavigationThread()
    navThread.start()
    camThread = CameraThread()
    camThread.start()

    while True:
        testCounter += 1
        time.sleep(0.01)  # threads don't improve CPU performance (only IO)
        # need to use processes later.
        currTime = time.time()
        if currTime - lastTime > 5:
            print('mainThread: ' + str(currTime) + ', ' + str(testCounter))
            lastTime = currTime
