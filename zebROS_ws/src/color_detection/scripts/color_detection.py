#!/usr/bin/env python

import rospy
from as726x_msgs.msg import AS726xCalibratedChannelData as ccd
from std_msgs.msg import Int8 
import numpy as np
from tensorflow.keras.models import load_model

model = load_model("src/color_detection/scripts/detect_adv.h5")
pub = rospy.Publisher("color_detected", Int8, queue_size=1000)

def callback(data):
    global pub
    data = np.expand_dims(np.array(data.calibrated_channel_data), axis=0)
    out = model.predict(data)[0]
    out = np.argmax(out)
    rospy.loginfo(str(out))
    pub.publish(out)

    
def listener():
    rospy.init_node("color_detect", anonymous=True)
    sub = rospy.Subscriber("color_request", ccd, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()

