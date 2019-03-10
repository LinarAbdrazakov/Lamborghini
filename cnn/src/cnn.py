import logging
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import numpy as np
import cv2
import tensorflow as tf
from keras.models import Sequential
from keras.layers import Dense, Flatten, Activation
from keras.layers import Dropout, Lambda
from keras.layers.convolutional import Convolution2D, MaxPooling2D
from keras.optimizers import Adam


print "[INFO] loading model..."
model = Sequential()
model.add(Lambda(lambda x: x / 127.5 - 1.0, input_shape=(32, 64, 3)))

model.add(Convolution2D(24, (5, 5), border_mode='same', subsample=(2, 2)))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2), strides=(1, 1)))

model.add(Convolution2D(36, (5, 5), border_mode='same', subsample=(2, 2)))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2), strides=(1, 1)))

model.add(Convolution2D(48, 5, 5, border_mode='same', subsample=(2, 2)))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2), strides=(1, 1)))

model.add(Convolution2D(64, 3, 3, border_mode='same', subsample=(1, 1)))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2), strides=(1, 1)))

model.add(Convolution2D(64, 3, 3, border_mode='same', subsample=(1, 1)))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2), strides=(1, 1)))

model.add(Flatten())
# Next, five fully connected layers
model.add(Dense(1164))
model.add(Activation('relu'))

model.add(Dense(100))
model.add(Activation('relu'))

model.add(Dense(50))
model.add(Activation('relu'))

model.add(Dense(10))
model.add(Activation('relu'))

model.add(Dense(1))

model.summary()

model.load_weights("lamba.h5")

print "[INFO] ok."
model.compile(optimizer=Adam(0.0001), loss="mse")

graph = tf.get_default_graph()


rospy.init_node("cnn_node")
pub = rospy.Publisher("angle_topic", Int32, queue_size=1)


def predict_angle(image):
    global model
    global graph
    with graph.as_default():
        road = image[-320:,:].copy()
        road = cv2.resize(road, (road.shape[1]//10, road.shape[0]//10), interpolation=cv2.INTER_AREA)

        road = np.array(road, "float").reshape(1, 32, 64, 3)
        angle = int(round(model.predict(road)[0][0] * 30 + 90))
        return angle


def callback(data):
    global pub
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    angle = predict_angle(image)
    angle_msg = Int32
    angle_msg = angle
    pub.publish(angle_msg)


sub = rospy.Subscriber("camera_topic", Image, callback)

if __name__ == "__main__":
    rospy.spin()

