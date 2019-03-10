from picamera.array import PiRGBArray
from picamera import PiCamera
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import logging
import time
import cv2


logging.basicConfig(
    format=u"%(levelname)-8s [%(asctime)s] %(message)s",
    level=logging.DEBUG
)


camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)


logging.info("Camera initialized")


rospy.init_node("camera_node")
pub = rospy.Publisher("camera_topic", Image, queue_size=1)

logging.info("Created node and publisher")

bridge = CvBridge()


past_time_fps = time.time()
number_cupture = 0
n_cuptures_for_fps = 10

if __name__ == "__main__":
    try:
        logging.info("Start stream")
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            image = frame.array
            pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))

            if rospy.is_shutdown():
                break

            number_cupture += 1
            if (number_cupture == n_cuptures_for_fps):
                print("FPS: " + str(round(n_cuptures_for_fps / (time.time() - past_time_fps), 2)))
                number_cupture = 0
                past_time_fps = time.time()

            rawCapture.truncate(0)

    finally:
        rawCapture.truncate(0)
