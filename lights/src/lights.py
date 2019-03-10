import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


class Lights:
    def __init__(self):
        self.forward_pin = 23
        self.ago_pin     = 24
        GPIO.setup(self.forward_pin, GPIO.OUT)
        GPIO.setup(self.ago_pin, GPIO.OUT)

    def forward(self, val):
        GPIO.output(self.forward_pin, val)

    def ago(self, val):
        GPIO.output(self.ago_pin, val)


def callback(msg):
    global lights
    print(msg.data)
    lights.forward(('f' in msg.data))
    lights.ago(('a' in msg.data))


if __name__ == "__main__":
    try:
        lights = Lights()
        rospy.init_node("lights_node")
        rospy.Subscriber("lights_topic", String, callback)
        rospy.spin()

    finally:
        GPIO.cleanup()
