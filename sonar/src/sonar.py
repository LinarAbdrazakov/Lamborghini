import time
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Int32


GPIO.setmode(GPIO.BCM)


class HC_SR04(object):
    def __init__(self):
        self.TRIG = 3
        self.ECHO = 2

        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)

    def get_dist(self):
        start = time.time()
        GPIO.output(self.TRIG, True)
        time.sleep(0.00001)
        GPIO.output(self.TRIG, False)
        while GPIO.input(self.ECHO) == False:
            pass
        start = time.time()
        while GPIO.input(self.ECHO) == True:
            pass
        stop = time.time()
        distance = round((stop - start) * 17000, 2)

        return distance


if __name__ == "__main__":
    try:
        sonar = HC_SR04()
        rospy.init_node("sonar_node")
        pub = rospy.Publisher("sonar_topic", Int32, queue_size=1)
        rate = rospy.Rate(25)
        while not rospy.is_shutdown():
            distance = sonar.get_dist()
            pub.publish(distance)
            rate.sleep()

    finally:
        GPIO.cleanup()
