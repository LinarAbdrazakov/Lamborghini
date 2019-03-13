import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import time


class State:
    def __init__(self):
        self.distance = 0
        self.angle    = 90
        self.battery  = None
        self.lights   = {"forward": 1,
                         "ago":     0}
        self.speed    = 0
        self.receiver_signal = None


class Controller:
    def __init__(self):
        rospy.init_node("control_node")
        # Subscribers
        self.sub_sonar    = rospy.Subscriber("sonar_topic", Float32, self.sonar_callback)
        self.sub_cnn      = rospy.Subscriber("cnn_topic", Int32, self.cnn_callback)
        self.sub_receiver = rospy.Subscriber("receiver_topic", Float32MultiArray, self.receiver_callback)
        self.sub_battery  = rospy.Subscriber("battery_state_topic", Float32, self.battery_callback)
        # Publishers
        self.pub_motor    = rospy.Publisher("motor_topic", Int32, queue_size=1)
        self.pub_lights   = rospy.Publisher("lights_topic", String, queue_size=1)
        self.pub_steering = rospy.Publisher("steering_topic", Int32, queue_size=1)
        # State machine
        self.state = State()
        self.porog_dist = 40

    def start_control(self):
        rospy.spin()

    def any_none(self):
        return (self.state.distance is None) or (self.state.angle is None) or (self.state.battery is None)

    def control(self):
        if self.state.receiver_signal is None:
            print "No signal from receiver!"
            return
        # angle
        self.state.angle = (self.state.receiver_signal[3] - 1500) / 25 + 90
        # converting signal from receiver to speed
        #print self.state.receiver_signal
        print self.state.speed
        self.state.speed = (self.state.receiver_signal[2] - 1500) / 5
        if self.state.speed > 100: self.state.speed = 100
        elif self.state.speed < -100: self.state.speed = -100
        # check safe distance
        if self.state.distance < self.porog_dist and self.state.speed > 0:
            self.state.speed = 0
            self.porog_dist  = 50
            self.state.lights["ago"] = 1
        else:
            self.porog_dist = 40
            self.state.ago = 1
            if self.state.speed >= 0:
                self.state.lights["ago"] = 0
            else:
                self.state.lights["ago"] = 1
        self.publish_all()

    def publish_all(self):
        if self.any_none():
           print "None", time.time()
           print self.state.distance, self.state.angle, self.state.battery
        #print "OK"
        self.publish_to_motor_topic()
        self.publish_to_lights_topic()
        self.publish_to_steering_topic()

    def publish_to_motor_topic(self):
        msg_speed = Int32()
        msg_speed.data = self.state.speed
        self.pub_motor.publish(msg_speed)

    def publish_to_lights_topic(self):
        msg_lights = String()
        msg_lights.data = ''
        msg_lights.data += "f" if self.state.lights["forward"] else ''
        msg_lights.data += "a" if self.state.lights["ago"] else ''
        self.pub_lights.publish(msg_lights)

    def publish_to_steering_topic(self):
        msg_angle = Int32()
        msg_angle.data = self.state.angle
        self.pub_steering.publish(msg_angle)


    def sonar_callback(self, msg):
	self.state.distance = msg.data
        self.control()

    def cnn_callback(self, msg):
        self.state.angle = msg.data
        self.control()

    def battery_callback(self, msg):
        self.state.battery = msg.data
        self.control()

    def receiver_callback(self, msg):
        self.state.receiver_signal = msg.data
        self.control()


if __name__ == "__main__":
    controller = Controller()
    controller.start_control()
