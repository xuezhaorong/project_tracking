import RPi.GPIO as GPIO
from time import sleep


class ServoCtrl:
    def __init__(self,servo,angle):
        self.servo = servo # servo
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(servo, GPIO.OUT)
        self.Servo_SetAngle(angle)

    def Servo_SetAngle(self,angle):
        pwm = GPIO.PWM(self.servo, 50)
        pwm.start(8)
        dutyCycle = angle / 18.0 + 3.0
        pwm.ChangeDutyCycle(dutyCycle)
        sleep(0.3)
        pwm.stop()

    def Servo_Debug(self,angle):
        for i in range(30, 150, 15):
            self.Servo_SetAngle(i)

        for i in range(150, 30, -15):
            self.Servo_SetAngle(i)

        self.Servo_SetAngle(angle)

    @classmethod
    def Servo_clean(self):
        GPIO.cleanup()

