#!/usr/bin/env python

import rospy
from as726x_msgs.msg import AS726CalibratedChannelData as ccd
from std_msgs.msg impot Int8, 
import numpy as np
from keras.models import load_model

model = load_model("detect_adv.h5")
pub = rospy.Publisher("color_detected", Int8, queue_size=1000)

def callback(data):
    global pub
    data = np.array(data.calibrated_channel_data).expand_dims(axis=0)
    out = model.predict(data)[0]
    pub.publish(out)

    
def listener():
    rospy.init_node("color_detect", anonymous=True)
    sub = rospy.Subscriber("color_request", ccd, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()

