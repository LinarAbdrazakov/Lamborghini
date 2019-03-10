import rospy
from std_msgs.msg import Int32
import RPi.GPIO as GPIO


GPIO.setmode(GPIO.BCM)


class Motor(object):
    def __init__(self):
        self.SpeedPin = 12
        self.ForwardPin = 6
        self.AgoPin = 5
        GPIO.setup(self.SpeedPin, GPIO.OUT)
        GPIO.setup(self.ForwardPin, GPIO.OUT)
        GPIO.setup(self.AgoPin, GPIO.OUT)
        self.Speed = GPIO.PWM(self.SpeedPin, 500)
        self.Speed.start(0)

    def go(self, speed):
        forward = False
        if speed > 0:
            forward = True
        GPIO.output(self.ForwardPin, forward)
        GPIO.output(self.AgoPin, not forward)
        self.Speed.ChangeDutyCycle(abs(speed))

    def stop(self):
        self.Speed.ChangeDutyCycle(0)
        self.Speed.stop()


def callback(speed):
    global motor
    motor.go(speed.data)


if __name__ == "__main__":
    try:
        motor = Motor()
        rospy.init_node("motor_node")
        rospy.Subscriber("motor_topic", Int32, callback)
        rospy.spin()

    finally:
        motor.stop()
        GPIO.cleanup()
