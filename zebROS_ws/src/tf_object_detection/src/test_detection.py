#!/usr/bin/env python

import cv2
import glob
import numpy as np
import os
import rospkg
import sys
import tensorflow as tf

import rospy
from sensor_msgs.msg import Image
from field_obj.msg import Detection, Object
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

detection_graph, sess, pub = None, None, None

# This is needed since the modules are in a subdir of
# the python script
# These are 'borrowed' from the tensorflow models object
# detection directory
from os.path import dirname, abspath, join
rospack = rospkg.RosPack()
THIS_DIR = rospack.get_path('tf_object_detection') + '/src/'
CODE_DIR = abspath(join(THIS_DIR, 'modules'))
sys.path.append(CODE_DIR)
from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

sub_topic = "c920/rect_image"
pub_topic = "tf_obj_detection"

# Takes a image, and using the tensorflow session and graph
# provided, runs inference on the image. This returns a list
# of detections - each includes the object bounding box, type
# and confidence
def run_inference_for_single_image(msg):
  image_np = bridge.imgmsg_to_cv2(msg, "rgb8")
  try:
    image = np.expand_dims(image_np, axis=0)
  except CvBridgeError as e:
    print(e)

  with detection_graph.as_default():
    # Get handles to input and output tensors
    ops = tf.compat.v1.get_default_graph().get_operations()
    all_tensor_names = {output.name for op in ops for output in op.outputs}
    tensor_dict = {}
    for key in [
        'num_detections', 'detection_boxes', 'detection_scores',
        'detection_classes', 'detection_masks'
    ]:
      tensor_name = key + ':0'
      if tensor_name in all_tensor_names:
        tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(
            tensor_name)
    if 'detection_masks' in tensor_dict:
      # The following processing is only for single image
      detection_boxes = tf.squeeze(tensor_dict['detection_boxes'], [0])
      detection_masks = tf.squeeze(tensor_dict['detection_masks'], [0])
      # Reframe is required to translate mask from box coordinates to image coordinates and fit the image size.
      real_num_detection = tf.cast(tensor_dict['num_detections'][0], tf.int32)
      detection_boxes = tf.slice(detection_boxes, [0, 0], [real_num_detection, -1])
      detection_masks = tf.slice(detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
      detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
          detection_masks, detection_boxes, image.shape[1], image.shape[2])
      detection_masks_reframed = tf.cast(
          tf.greater(detection_masks_reframed, 0.5), tf.uint8)
      # Follow the convention by adding back the batch dimension
      tensor_dict['detection_masks'] = tf.expand_dims(
          detection_masks_reframed, 0)
    image_tensor = tf.compat.v1.get_default_graph().get_tensor_by_name('image_tensor:0')

    # Run inference
    output_dict = sess.run(tensor_dict,
                           feed_dict={image_tensor: image})

    # all outputs are float32 numpy arrays, so convert types as appropriate
    num_detections = int(output_dict['num_detections'][0])
    detection_classes = output_dict['detection_classes'][0].astype(np.int64)
    detection_boxes = output_dict['detection_boxes'][0]
    detection_scores = output_dict['detection_scores'][0]
    if 'detection_masks' in output_dict:
        detection_masks = output_dict['detection_masks'][0]

    detection = Detection()
    for i in range(num_detections):
        obj = Object()
        obj.location.x = (detection_boxes[i][0] + detection_boxes[i][1]) / 2
        obj.location.y = (detection_boxes[i][2] + detection_boxes[i][3]) / 2
        obj.confidence = detection_scores[i]
        obj.id = str(detection_classes[i])
        detection.objects.append(obj)

    pub.publish(detection)

def vis(output_dict):
    vis_util.visualize_boxes_and_labels_on_image_array(
        image_np,
        detection_boxes,
        detection_classes,
        detection_scores,
        category_index,
        instance_masks= detection_masks,
        use_normalized_coordinates=True,
        line_thickness=4,
        max_boxes_to_draw=50,
        min_score_thresh=0.35,
        groundtruth_box_visualization_color='yellow')

    cv2.imshow('img', cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR))
    key = cv2.waitKey(5) & 0xFF

def main():
    global THIS_DIR
    global detection_graph, sess, pub
    print THIS_DIR

    rospy.init_node('tf_object_detection', anonymous = True)

    # Path to frozen detection graph. This is the actual model that is used for the object detection.
    # This shouldn't need to change
    PATH_TO_FROZEN_GRAPH = THIS_DIR + 'frozen_inference_graph.pb'

    # List of the strings that is used to add correct label for each box.
    PATH_TO_LABELS = THIS_DIR + '2020Game_label_map.pbtxt'
    category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)

    # Init TF detection graph and session
    detection_graph = tf.Graph()
    with detection_graph.as_default():
      od_graph_def = tf.GraphDef()
      with tf.gfile.GFile(PATH_TO_FROZEN_GRAPH, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

    with detection_graph.as_default():
        sess = tf.Session(graph=detection_graph)

    sub = rospy.Subscriber(sub_topic, Image, run_inference_for_single_image)
    pub = rospy.Publisher(pub_topic, Detection, queue_size=10)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
