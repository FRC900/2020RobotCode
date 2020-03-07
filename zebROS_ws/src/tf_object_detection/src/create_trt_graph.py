'''
Script to generate TRT output graph files from a given checkpoint
TensorRT generates optimized code for running networks on GPUs, both for
desktop and especially for the Jetson.  The optimization process takes a bit
of time, so it is possible to preprocess and save the output.
That's the goal of this script.
Edit the SAVED_MODEL_DIR and CHECKPOINT_NUMBER variables below and run it.
'''
import tensorflow as tf
import tensorflow.contrib.tensorrt as trt
from object_detection.protos import pipeline_pb2
from object_detection import exporter
from google.protobuf import text_format
import os
import subprocess

from graph_utils import force_nms_cpu as f_force_nms_cpu
from graph_utils import replace_relu6 as f_replace_relu6
from graph_utils import remove_assert as f_remove_assert

from file_changed.py import file_changed

# Output file name
TRT_OUTPUT_GRAPH = 'trt_graph.pb'

# Dir where model.ckpt* files are being generated
SAVED_MODEL_DIR='/home/ubuntu/2020RobotCode/zebROS_ws/src/tf_object_detection/src'

INPUT_NAME='image_tensor'
BOXES_NAME='detection_boxes'
CLASSES_NAME='detection_classes'
SCORES_NAME='detection_scores'
MASKS_NAME='detection_masks'
NUM_DETECTIONS_NAME='num_detections'
MODEL_CHECKPOINT_PREFIX='model.ckpt-'
FROZEN_GRAPH_NAME='frozen_inference_graph.pb'
OUTPUT_DIR='/home/ubuntu/2020RobotCode/zebROS_ws/src/tf_object_detection/src'

# from tf_trt models dir
def build_detection_graph(
        force_nms_cpu=True,
        replace_relu6=True,
        remove_assert=True,
        input_shape=None,
        output_dir=SAVED_MODEL_DIR):
    """Builds a frozen graph for a pre-trained object detection model"""

    # read frozen graph from file
    frozen_graph = tf.GraphDef()
    with open(os.path.join(output_dir, FROZEN_GRAPH_NAME), 'rb') as f:
        frozen_graph.ParseFromString(f.read())

    # apply graph modifications
    if force_nms_cpu:
        frozen_graph = f_force_nms_cpu(frozen_graph)
    if replace_relu6:
        frozen_graph = f_replace_relu6(frozen_graph)
    if remove_assert:
        frozen_graph = f_remove_assert(frozen_graph)

    # get input names
    # TODO: handle mask_rcnn
    input_names = [INPUT_NAME]
    output_names = [BOXES_NAME, CLASSES_NAME, SCORES_NAME, NUM_DETECTIONS_NAME]

    # remove temporary directory
    subprocess.call(['rm', '-rf', output_dir])

    return frozen_graph, input_names, output_names


def main():
    # What model to run from - should be the directory name of an exported trained model
    # Change me to the directory checkpoint files are saved in

    if not file_changed(os.path.join(SAVED_MODEL_DIR, FROZEN_GRAPH_NAME)):
        return

    frozen_graph, input_names, output_names = build_detection_graph(
    )
    trt_graph = trt.create_inference_graph(
        input_graph_def=frozen_graph,
        outputs=output_names,
        max_batch_size=1,
        max_workspace_size_bytes=1 << 25,
        precision_mode='FP32', # TODO - FP16 or INT8 for Jetson
        minimum_segment_size=50
    )

    with open(os.path.join(SAVED_MODEL_DIR, TRT_OUTPUT_GRAPH), 'wb') as f:
        f.write(trt_graph.SerializeToString())

if __name__ == '__main__':
    main()
